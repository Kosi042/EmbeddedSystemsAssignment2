set(SRC
	ximge_processor.c
)

set(HEADERS
	ximge_processor.h
	ximge_processor_hw.h
	ximge_processor_linux.c
)

add_library(drivers ${SRC} ${HEADERS})

