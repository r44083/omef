set(TARGET fatfs)

add_library(${TARGET} STATIC
    ff.c
    ffsystem.c
    ffunicode.c
)
