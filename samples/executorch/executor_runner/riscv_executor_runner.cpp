/*
	modified from executorch/examples/arm/executor_runner/arm_executor_runner.cpp to run on riscv
*/

#include <errno.h>
#include <memory>
#include <stdio.h>
#include <unistd.h>
#include <vector>

#include <executorch/extension/data_loader/buffer_data_loader.h>
#include <executorch/extension/runner_util/inputs.h>
#include <executorch/runtime/core/memory_allocator.h>
#include <executorch/runtime/executor/program.h>
#include <executorch/runtime/platform/log.h>
#include <executorch/runtime/platform/platform.h>
#include <executorch/runtime/platform/runtime.h>

#if defined(__ZEPHYR__)
#include <zephyr/kernel.h>
#include <zephyr/sys/reboot.h>
#endif

struct _reent * _impure_ptr = nullptr;

void *__dso_handle = nullptr;

#ifdef MB_MULTI_MODEL
#include "models_pte.h"   /* registry of N baked .pte models (batched run) */
#else
#include "model_pte.h"
#endif

/* Opt-in TACIT / L-Trace of just the model execute() (the final, warm
 * iteration). Build with -DMB_TACIT_TRACE_MODEL=1 and run on the TACIT-enabled
 * spike with `--trace=l`. See samples/tacit/TACIT_TRACING.md. */
#if defined(MB_TACIT_TRACE_MODEL)
extern "C" {
#include <tacit/tacit.h>
}
#endif

/* How many times to run method->execute() for cold-vs-warm cycle profiling.
 * Override with -DMB_EXEC_ITERS=N. */
#ifndef MB_EXEC_ITERS
#define MB_EXEC_ITERS 5
#endif

using executorch::aten::ScalarType;
using executorch::aten::Tensor;
using executorch::aten::TensorImpl;
using executorch::extension::BufferCleanup;
using executorch::extension::BufferDataLoader;
using executorch::runtime::Error;
using executorch::runtime::EValue;
using executorch::runtime::HierarchicalAllocator;
using executorch::runtime::MemoryAllocator;
using executorch::runtime::MemoryManager;
using executorch::runtime::Method;
using executorch::runtime::MethodMeta;
using executorch::runtime::Program;
using executorch::runtime::Result;
using executorch::runtime::Span;
using executorch::runtime::Tag;
using executorch::runtime::TensorInfo;

/**
 * The method_allocation_pool should be large enough to fit the setup, input
 * used and other data used like the planned memory pool (e.g. memory-planned
 * buffers to use for mutable tensor data).
 */
// Sized to hold the (largest, when batched) model's planned activations +
// setup + inputs. Bump via -DMB_ET_POOL_MB for the utilization-aware default
// sizing (256 MB baked io needs >96 MB of activations). Needs ram0 to hold it.
#ifndef MB_ET_POOL_MB
#define MB_ET_POOL_MB 96
#endif
const size_t method_allocation_pool_size = (size_t)MB_ET_POOL_MB * 1024 * 1024;
// .bss (zeroed, NOLOAD): the previous `input_data_sec` was a LOADED section, so
// MB_ET_POOL_MB of zeros got baked into the ELF and streamed over the (per-word
// MMIO) FireSim loader — a 256 MB pool turned a ~12 MB code image into a 268 MB
// load (~15 min). Plain .bss is NOLOAD (only real code/model bytes transfer) AND
// boot-zeroed. Zeroing matters: the ET allocator/planner reads pool memory that
// must start zero — an unzeroed (.noinit) pool faults with a NULL-deref
// (mcause 5, mtval 0) mid-execute once allocations succeed.
unsigned char
	__attribute__((aligned(16))) method_allocation_pool[method_allocation_pool_size];

/**
 * The temp_allocation_pool is used for allocating temporary data during kernel
 * or delegate execution. This will be reset after each kernel or delegate call.
 * Currently a MemoryAllocator is used but a PlatformMemoryAllocator is probably
 * a better fit
 */
#ifndef MB_ET_TEMP_MB
#define MB_ET_TEMP_MB 1
#endif
const size_t temp_allocation_pool_size = (size_t)MB_ET_TEMP_MB * 1024 * 1024;
unsigned char __attribute__((aligned(16))) temp_allocation_pool[temp_allocation_pool_size];

#ifdef MB_TACIT_TRACE_DMA
/* On FireSim the TACIT encoder can't print over HTIF (TARGET_PRINT is a spike-sim
 * hook); it streams to DRAM via the TraceSinkDMA. We reserve a fixed .bss buffer
 * here — its address is a stable ELF symbol the HOST driver reads back after the
 * run (`+dump-mem=<addr>:<len>:<file>`), so the trace survives even a target
 * crash (the DMA already landed the bytes). See samples/tacit/TACIT_TRACING.md. */
#ifndef MB_TACIT_DMA_BUF_MB
#define MB_TACIT_DMA_BUF_MB 32
#endif
#ifndef MB_TACIT_DMA_BYPASS
#define MB_TACIT_DMA_BYPASS 0   /* set 1 if the DMA sink needs the SBUS bypass path */
#endif
unsigned char __attribute__((aligned(64)))
	mb_tacit_dma_buf[(size_t)MB_TACIT_DMA_BUF_MB * 1024 * 1024];
#endif

/* void et_pal_init(void)
{
} */

ET_NORETURN void et_pal_abort(void)
{
#ifndef SEMIHOSTING
	__builtin_trap();
#else
	_exit(-1);
#endif
}

/**
 * Emit a log message via platform output (serial port, console, etc).
 */
void et_pal_emit_log_message(ET_UNUSED et_timestamp_t timestamp, et_pal_log_level_t level, const char *filename,
							 ET_UNUSED const char *function, size_t line, const char *message, ET_UNUSED size_t length)
{
	fprintf(stderr, "%c [executorch:%s:%zu] %s\n", level, filename, line, message);
}

namespace
{

// Setup our own allocator that can show some extra stuff like used and free memory info
class RiscvMemoryAllocator : public executorch::runtime::MemoryAllocator
{
public:
	RiscvMemoryAllocator(uint32_t size, uint8_t *base_address) : MemoryAllocator(size, base_address), used_(0)
	{
	}

	void *allocate(size_t size, size_t alignment = kDefaultAlignment) override
	{
		void *ret = executorch::runtime::MemoryAllocator::allocate(size, alignment);
		if (ret != nullptr)
		{
			// Align with the same code as in MemoryAllocator::allocate() to keep
			// used_ "in sync" As alignment is expected to be power of 2 (checked by
			// MemoryAllocator::allocate()) we can check it the lower bits
			// (same as alignment - 1) is zero or not.
			if ((size & (alignment - 1)) == 0)
			{
				// Already aligned.
				used_ += size;
			}
			else
			{
				used_ = (used_ | (alignment - 1)) + 1 + size;
			}
		}
		return ret;
	}

	// Returns the used size of the allocator's memory buffer.
	size_t used_size() const
	{
		return used_;
	}

	// Returns the free size of the allocator's memory buffer.
	size_t free_size() const
	{
		return executorch::runtime::MemoryAllocator::size() - used_;
	}

private:
	size_t used_;
};

Result<BufferCleanup> prepare_input_tensors(Method &method, MemoryAllocator &allocator,
											std::vector<std::pair<char *, size_t>> &input_buffers)
{
	MethodMeta method_meta = method.method_meta();
	size_t num_inputs = method_meta.num_inputs();
	size_t num_allocated = 0;

	void **inputs = static_cast<void **>(allocator.allocate(num_inputs * sizeof(void *)));

	ET_CHECK_OR_RETURN_ERROR(inputs != nullptr, MemoryAllocationFailed,
							 "Could not allocate memory for pointers to input buffers.");

	for (size_t i = 0; i < num_inputs; i++)
	{
		auto tag = method_meta.input_tag(i);
		ET_CHECK_OK_OR_RETURN_ERROR(tag.error());

		if (tag.get() != Tag::Tensor)
		{
			ET_LOG(Debug, "Skipping non-tensor input %zu", i);
			continue;
		}
		Result<TensorInfo> tensor_meta = method_meta.input_tensor_meta(i);
		ET_CHECK_OK_OR_RETURN_ERROR(tensor_meta.error());

		// Input is a tensor. Allocate a buffer for it.
		void *data_ptr = allocator.allocate(tensor_meta->nbytes());
		ET_CHECK_OR_RETURN_ERROR(data_ptr != nullptr, MemoryAllocationFailed,
								 "Could not allocate memory for input buffers.");
		inputs[num_allocated++] = data_ptr;

		Error err = Error::Ok;
		if (input_buffers.size() > 0)
		{
			auto [buffer, buffer_size] = input_buffers.at(i);
			if (buffer_size != tensor_meta->nbytes())
			{
				ET_LOG(Error, "input size (%d) and tensor size (%d) missmatch!", buffer_size, tensor_meta->nbytes());
				err = Error::InvalidArgument;
			}
			else
			{
				ET_LOG(Info, "Copying read input to tensor.");
				std::memcpy(data_ptr, buffer, buffer_size);
			}
		}

		TensorImpl impl = TensorImpl(tensor_meta.get().scalar_type(), tensor_meta.get().sizes().size(),
									 const_cast<TensorImpl::SizesType *>(tensor_meta.get().sizes().data()), data_ptr,
									 const_cast<TensorImpl::DimOrderType *>(tensor_meta.get().dim_order().data()));
		Tensor t(&impl);

		// If input_buffers.size <= 0, we don't have any input, fill t with 1's.
		if (input_buffers.size() <= 0)
		{
			for (size_t j = 0; j < t.numel(); j++)
			{
				switch (t.scalar_type())
				{
				case ScalarType::Int:
					t.mutable_data_ptr<int>()[j] = 1;
					break;
				case ScalarType::Float:
					t.mutable_data_ptr<float>()[j] = 1.;
					break;
				}
			}
		}

#ifdef MB_MEMDBG
		// EXPERIMENT (ITERS=1 zero-output culprit): input-side fence + scalar
		// readback. If adding this fence alone makes convs correct, the failing
		// edge is scalar-input-fill -> vector-load visibility. The printed addr
		// vs the output addr (MB_OUTPUT_DBG) tests buffer aliasing.
		__asm__ volatile("fence rw, rw" ::: "memory");
		if (t.scalar_type() == ScalarType::Float && t.numel() > 0) {
			const float* ip = t.const_data_ptr<float>();
			volatile float v0 = ip[0], vm = ip[t.numel()/2], vl = ip[t.numel()-1];
			printf("MB_INPUT_DBG idx=%zu addr=%p numel=%zu in[0]=%f in[mid]=%f in[last]=%f\n",
			       (size_t)i, (const void*)ip, (size_t)t.numel(), (double)v0, (double)vm, (double)vl);
			fflush(stdout);
		}
#endif

		err = method.set_input(t, i);

		if (err != Error::Ok)
		{
			ET_LOG(Error, "Failed to prepare input %zu: 0x%" PRIx32, i, (uint32_t)err);
			// The BufferCleanup will free the inputs when it goes out of scope.
			BufferCleanup cleanup({inputs, num_allocated});
			return err;
		}
	}
	return BufferCleanup({inputs, num_allocated});
}

} // namespace

// Run one baked .pte end-to-end: load the program, prepare inputs, execute
// MB_EXEC_ITERS times (rdcycle-bracketed), and print `_mb_tag`-tagged cycles +
// an output checksum. All allocators are constructed fresh on entry, so they
// reset the shared static pools — a multi-model loop reuses one pool
// sequentially (only one model's activations are live at a time).
static int run_one_pte(const unsigned char *model_pte, unsigned int model_pte_size, const char *_mb_tag)
{
	std::vector<std::pair<char *, size_t>> input_buffers;
	size_t pte_size = model_pte_size;

	ET_LOG(Info, "Model in %p %c", model_pte, model_pte[0]);
	auto loader = BufferDataLoader(model_pte, pte_size);
	ET_LOG(Info, "Model PTE file loaded. Size: %lu bytes.", pte_size);
	// Leak the ExecuTorch objects (program/method/inputs): their destructors
	// tear down the XNNPACK delegate + shared workspace, which triggers a
	// Load access fault on this bare-metal build — the SAME fault that's benign
	// at end-of-program in single-model mode, but here it fires after every
	// model and kills the loop. Each model uses a fresh allocator on the reset
	// pool and we never revisit a prior model, so never destructing them is safe
	// (and lets model N+1 reuse the shared workspace instead of re-initing it).
	Result<Program> &program = *(new Result<Program>(Program::load(&loader)));
	if (!program.ok())
	{
		ET_LOG(Info, "Program loading failed @ 0x%p: 0x%" PRIx32, model_pte, program.error());
	}

	ET_LOG(Info, "Model buffer loaded, has %lu methods", program->num_methods());

	const char *method_name = nullptr;
	{
		const auto method_name_result = program->get_method_name(0);
		ET_CHECK_MSG(method_name_result.ok(), "Program has no methods");
		method_name = *method_name_result;
	}
	ET_LOG(Info, "Running method %s", method_name);

	Result<MethodMeta> method_meta = program->method_meta(method_name);
	if (!method_meta.ok())
	{
		ET_LOG(Info, "Failed to get method_meta for %s: 0x%x", method_name, (unsigned int)method_meta.error());
	}

	ET_LOG(Info, "Setup Method allocator pool. Size: %lu bytes.", method_allocation_pool_size);

	RiscvMemoryAllocator method_allocator(method_allocation_pool_size, method_allocation_pool);

	std::vector<uint8_t *> planned_buffers;	  // Owns the memory
	std::vector<Span<uint8_t>> planned_spans; // Passed to the allocator
	size_t num_memory_planned_buffers = method_meta->num_memory_planned_buffers();

	size_t planned_buffer_membase = method_allocator.used_size();

	for (size_t id = 0; id < num_memory_planned_buffers; ++id)
	{
		size_t buffer_size = static_cast<size_t>(method_meta->memory_planned_buffer_size(id).get());
		ET_LOG(Info, "Setting up planned buffer %zu, size %zu.", id, buffer_size);

		/* Move to it's own allocator when MemoryPlanner is in place. */
		uint8_t *buffer = reinterpret_cast<uint8_t *>(method_allocator.allocate(buffer_size));
		planned_buffers.push_back(buffer);
		planned_spans.push_back({planned_buffers.back(), buffer_size});
	}

	size_t planned_buffer_memsize = method_allocator.used_size() - planned_buffer_membase;

	HierarchicalAllocator planned_memory({planned_spans.data(), planned_spans.size()});

	RiscvMemoryAllocator temp_allocator(temp_allocation_pool_size, temp_allocation_pool);

	MemoryManager memory_manager(&method_allocator, &planned_memory, &temp_allocator);

	size_t method_loaded_membase = method_allocator.used_size();

	Result<Method> &method = *(new Result<Method>(
		program->load_method(method_name, &memory_manager)));   // leaked (see above)
	if (!method.ok())
	{
		ET_LOG(Info, "Loading of method %s failed with status 0x%" PRIx32, method_name, method.error());
	}
	size_t method_loaded_memsize = method_allocator.used_size() - method_loaded_membase;
	ET_LOG(Info, "Method loaded.");

	ET_LOG(Info, "Preparing inputs...");
	size_t input_membase = method_allocator.used_size();

	auto &inputs = *(new auto(::prepare_input_tensors(*method, method_allocator, input_buffers)));  // leaked (see above)

	if (!inputs.ok())
	{
		ET_LOG(Info, "Preparing inputs tensors for method %s failed with status 0x%" PRIx32, method_name,
			   inputs.error());
	}
	size_t input_memsize = method_allocator.used_size() - input_membase;
	ET_LOG(Info, "Input prepared.");

	ET_LOG(Info, "Starting the model execution...");
	size_t executor_membase = method_allocator.used_size();
	/* rdcycle bracket around execute() so the flow-comparison harness
	 * (compare_flows.sh) can parse a per-run cycle count that lines up with
	 * modelblaster's rdcycle totals. We run execute() MB_EXEC_ITERS times so
	 * the harness can separate cold (iter 0: XNNPACK runtime create + weight
	 * pack + memory plan) from warm (steady-state compute) cost — a single
	 * cold run wildly overstates the compute. Printed with printf (not ET_LOG)
	 * so it survives log-level filtering. */
	Error status = Error::Ok;
	const int _mb_iters = MB_EXEC_ITERS;
#if defined(MB_TACIT_TRACE_MODEL)
	LTraceEncoderType *_tacit_enc = l_trace_encoder_get(0);
#ifdef MB_TACIT_TRACE_DMA
	/* FireSim: point the encoder at the DMA sink, targeted at our .bss buffer. */
	LTraceSinkDmaType *_tacit_sink = l_trace_sink_dma_get(0);
	l_trace_sink_dma_configure_addr(_tacit_sink, (uint64_t)(uintptr_t)mb_tacit_dma_buf,
									MB_TACIT_DMA_BYPASS);
	l_trace_encoder_configure_target(_tacit_enc, TARGET_DMA);
#else
	l_trace_encoder_configure_target(_tacit_enc, TARGET_PRINT);
#endif
#endif
	for (int _it = 0; _it < _mb_iters; _it++) {
		unsigned long _mb_cyc_begin, _mb_cyc_end;
#if defined(MB_TACIT_TRACE_MODEL)
		/* Trace only the final, warm iteration (cold iter 0 includes XNNPACK
		 * runtime create + weight pack, which we don't want in the trace). */
		bool _tacit_on = (_it == _mb_iters - 1);
		if (_tacit_on) l_trace_encoder_start(_tacit_enc);
#endif
		// Re-establish inputs before EVERY execute (via set_input, which re-copies
		// into the method's PLANNED input buffers): the memory planner may reuse a
		// dead input buffer as scratch/output within an execute, so a repeated
		// execute would otherwise read stale output as input (the ITERS>1 warm
		// iterations were reading the previous output => wrong checksums). Writing
		// get_input()'s buffer is NOT enough — set_input is what lands in the plan.
#ifndef MB_NO_INPUT_REINIT
		if (_it > 0) {
			auto &_reinit = *(new auto(::prepare_input_tensors(*method, method_allocator, input_buffers)));  // leaked
			(void)_reinit;
		}
#endif
		__asm__ volatile("rdcycle %0" : "=r"(_mb_cyc_begin));
		status = method->execute();
		__asm__ volatile("rdcycle %0" : "=r"(_mb_cyc_end));
#if defined(MB_TACIT_TRACE_MODEL)
		if (_tacit_on) {
			l_trace_encoder_stop(_tacit_enc);
			for (int _i = 0; _i < 16; _i++) __asm__ volatile("nop"); /* flush */
#ifdef MB_TACIT_TRACE_DMA
			/* Drain the sink FIFO to DRAM, then announce the region so the host
			 * (+dump-mem) knows exactly what to read back. Printed BEFORE any
			 * post-execute work so it's on the console even if a later model
			 * faults; the DMA'd bytes are already in DRAM regardless. */
			_tacit_sink->TR_SK_DMA_FLUSH = 1;
			while (_tacit_sink->TR_SK_DMA_FLUSH_DONE == 0) { }
			unsigned long _tr_bytes = (unsigned long)_tacit_sink->TR_SK_DMA_COUNT;
			printf("MB_TACIT_DMA_TRACE tag=%s addr=0x%lx bytes=%lu bufsz=%lu\n",
				   _mb_tag, (unsigned long)(uintptr_t)mb_tacit_dma_buf, _tr_bytes,
				   (unsigned long)sizeof(mb_tacit_dma_buf));
			fflush(stdout);
#endif
		}
#endif
		printf("EXECUTORCH_EXECUTE_CYCLES[%s][%d]=%lu\n", _mb_tag, _it, _mb_cyc_end - _mb_cyc_begin);
		fflush(stdout);
	}
	// The final execute()'s outputs are written by the Saturn vector unit (RVV
	// stores). Make them visible to the scalar reads below (get_outputs/checksum)
	// with a full memory fence. Without this, a single iteration (MB_EXEC_ITERS=1)
	// reads all-zeros on FireSim; a 2nd iteration only "fixed" it by incidentally
	// forcing the prior stores through. (This SoC has no CBO/cache-mgmt config, so
	// rely on the RVV/scalar coherence + fence ordering.)
	__asm__ volatile("fence rw, rw" ::: "memory");
	size_t executor_memsize = method_allocator.used_size() - executor_membase;

	ET_LOG(Info, "model_pte_loaded_size:     %lu bytes.", pte_size);

	if (method_allocator.size() != 0)
	{
		size_t method_allocator_used = method_allocator.used_size();
		ET_LOG(Info, "method_allocator_used:     %zu / %zu  free: %zu ( used: %zu %% ) ", method_allocator_used,
			   method_allocator.size(), method_allocator.free_size(),
			   100 * method_allocator_used / method_allocator.size());
		ET_LOG(Info, "method_allocator_planned:  %zu bytes", planned_buffer_memsize);
		ET_LOG(Info, "method_allocator_loaded:   %zu bytes", method_loaded_memsize);
		ET_LOG(Info, "method_allocator_input:    %zu bytes", input_memsize);
		ET_LOG(Info, "method_allocator_executor: %zu bytes", executor_memsize);
	}
	if (temp_allocator.size() > 0)
	{
		ET_LOG(Info, "temp_allocator_used:       %zu / %zu free: %zu ( used: %zu %% ) ", temp_allocator.used_size(),
			   temp_allocator.size(), temp_allocator.free_size(),
			   100 * temp_allocator.used_size() / temp_allocator.size());
	}

	if (status != Error::Ok)
	{
		ET_LOG(Info, "Execution of method %s failed with status 0x%" PRIx32, method_name, status);
	}
	else
	{
		ET_LOG(Info, "Model executed successfully.");
	}

	std::vector<EValue> outputs(method->outputs_size());
	ET_LOG(Info, "%zu outputs: ", outputs.size());
	status = method->get_outputs(outputs.data(), outputs.size());
	ET_CHECK(status == Error::Ok);
	for (int i = 0; i < outputs.size(); ++i)
	{
		Tensor t = outputs[i].toTensor();
		const int n = (int)t.numel();
#ifdef MB_MEMDBG
		// Output buffer address (compare to MB_INPUT_DBG addr for aliasing) + a
		// scalar readback of the raw output region right here (before the checksum).
		if (t.scalar_type() == ScalarType::Float && n > 0) {
			const float* op = t.const_data_ptr<float>();
			printf("MB_OUTPUT_DBG idx=%d addr=%p numel=%d out[0]=%f out[mid]=%f out[last]=%f\n",
			       i, (const void*)op, n, (double)op[0], (double)op[n/2], (double)op[n-1]);
			fflush(stdout);
		}
#endif
		// The output might be collected and parsed so printf() is used instead
		// of ET_LOG() here.
		// FireSim's HTIF console costs ~millions of cycles per char, so dumping
		// every element (KernelBench outputs reach ~50k) dominates wall time and
		// hits the run timeout. By default emit a checksum + the first few
		// elements (enough to sanity-check / diff against the golden); define
		// MB_ET_FULL_OUTPUT to restore the full per-element dump.
#ifdef MB_ET_FULL_OUTPUT
		for (int j = 0; j < n; ++j)
		{
			if (t.scalar_type() == ScalarType::Int)
				printf("Output[%d][%d]: %d\n", i, j, t.const_data_ptr<int>()[j]);
			else
				printf("Output[%d][%d]: %f\n", i, j, t.const_data_ptr<float>()[j]);
		}
#else
		// The checksum alone is enough to validate against the host golden; the
		// first-few-element dump is pure HTIF overhead on FireSim (each line
		// ~millions of cycles). Gate it behind MB_ET_SAMPLE_OUTPUT for debugging.
		if (t.scalar_type() == ScalarType::Int)
		{
			const int *p = t.const_data_ptr<int>();
			long long sum = 0;
			for (int j = 0; j < n; ++j) sum += p[j];
			printf("Output[%d] numel=%d checksum=%lld\n", i, n, sum);
#ifdef MB_ET_SAMPLE_OUTPUT
#ifndef MB_ET_SAMPLE_N
#define MB_ET_SAMPLE_N 64
#endif
			{ int _sn = n < MB_ET_SAMPLE_N ? n : MB_ET_SAMPLE_N;
			  for (int j = 0; j < _sn; ++j) printf("O[%d][%d]=%d\n", i, j, p[j]);
			  int _st = n / 64; if (_st < 1) _st = 1;   // strided sample across the rest
			  for (int j = _sn; j < n; j += _st) printf("O[%d][%d]=%d\n", i, j, p[j]); }
#endif
		}
		else
		{
			const float *p = t.const_data_ptr<float>();
			double sum = 0.0;
			for (int j = 0; j < n; ++j) sum += (double)p[j];
			printf("Output[%d] numel=%d checksum=%f\n", i, n, sum);
#ifdef MB_ET_SAMPLE_OUTPUT
#ifndef MB_ET_SAMPLE_N
#define MB_ET_SAMPLE_N 64
#endif
			// Dump first MB_ET_SAMPLE_N output elems + a strided sample across the
			// rest, so a spike-vs-FireSim diff shows WHERE the kernel diverges.
			{ int _sn = n < MB_ET_SAMPLE_N ? n : MB_ET_SAMPLE_N;
			  for (int j = 0; j < _sn; ++j) printf("O[%d][%d]=%.6f\n", i, j, p[j]);
			  int _st = n / 64; if (_st < 1) _st = 1;
			  for (int j = _sn; j < n; j += _st) printf("O[%d][%d]=%.6f\n", i, j, p[j]); }
#endif
		}
#endif
	}
	printf("MB_MODEL_DONE_OUTPUT=%s\n", _mb_tag); fflush(stdout);
	return 0; /* end run_one_pte — locals (method/program/allocators) destruct here */
}

int main()
{
	executorch::runtime::runtime_init();
#ifdef MB_MULTI_MODEL
	// Batched: run every baked model sequentially under ONE boot (and, on
	// FireSim, ONE infrasetup). Every model's io lives in rodata; only the
	// active model's activations occupy the pool at a time (fresh allocators
	// per call reset it), so N models cost N*io of RAM + one activation set.
	for (unsigned _mi = 0; _mi < mb_num_models; ++_mi) {
		printf("MB_MODEL_BEGIN=%s\n", mb_models[_mi].name); fflush(stdout);
		run_one_pte(mb_models[_mi].data, mb_models[_mi].size, mb_models[_mi].name);
		printf("MB_MODEL_END=%s\n", mb_models[_mi].name); fflush(stdout);
	}
#else
	run_one_pte(model_pte, model_pte_size, "model");
#endif
	ET_LOG(Info, "Program complete, exiting.");
	ET_LOG(Info, "\04");
	// On FireSim/spike SMP, returning from main hits a benign load-fault on the
	// shutdown path and the sim never cleanly exits, pinning the FPGA until the
	// run timeout. Reboot for a clean tohost exit (mirrors modelblaster/harness).
#if defined(__ZEPHYR__)
	sys_reboot(SYS_REBOOT_COLD);
#endif
	return 0;
}
