#pragma once

#include <stddef.h>
#include <stdint.h>

/* Builds a NUL-terminated text line in a caller-owned buffer, without printf (nano libc has no
 * float formatting). Output that does not fit is cut off; the buffer always stays terminated. */
typedef struct {
    char *buffer;
    size_t used;
    size_t capacity;
} EasyTVCTextLine;

void EasyTVC_LineStr(EasyTVCTextLine *line, const char *text);
void EasyTVC_LineInt(EasyTVCTextLine *line, int32_t value);
/* "," then the number: one CSV field. */
void EasyTVC_LineField(EasyTVCTextLine *line, int32_t value);
/* "0x" and two upper-case hex digits. */
void EasyTVC_LineHex8(EasyTVCTextLine *line, uint8_t value);
