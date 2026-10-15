#include "text_line.h"

void EasyTVC_LineStr(EasyTVCTextLine *line, const char *text)
{
    while (*text != '\0' && line->used + 1U < line->capacity) {
        line->buffer[line->used++] = *text++;
    }
    line->buffer[line->used] = '\0';
}

void EasyTVC_LineInt(EasyTVCTextLine *line, int32_t value)
{
    char reversed[11];
    char text[12];
    size_t n = 0, used = 0;
    uint32_t magnitude = value < 0 ? 0U - (uint32_t)value : (uint32_t)value;
    do {
        reversed[n++] = (char)('0' + magnitude % 10U);
        magnitude /= 10U;
    } while (magnitude != 0U);
    if (value < 0) {
        text[used++] = '-';
    }
    while (n > 0U) {
        text[used++] = reversed[--n];
    }
    text[used] = '\0';
    EasyTVC_LineStr(line, text);
}

void EasyTVC_LineField(EasyTVCTextLine *line, int32_t value)
{
    EasyTVC_LineStr(line, ",");
    EasyTVC_LineInt(line, value);
}

void EasyTVC_LineHex8(EasyTVCTextLine *line, uint8_t value)
{
    static const char hex[] = "0123456789ABCDEF";
    const char text[5] = {'0', 'x', hex[value >> 4], hex[value & 15U], '\0'};
    EasyTVC_LineStr(line, text);
}
