#include <zephyr/linker/iterable_sections.h>

/* Per-service iterable ROM section for the HTTP resources registered on the `rb_http` service
 * (HTTP_SERVICE_DEFINE / HTTP_RESOURCE_DEFINE in main.c) -- provides the list start/end markers. */
ITERABLE_SECTION_ROM(http_resource_desc_rb_http, Z_LINK_ITERABLE_SUBALIGN)
