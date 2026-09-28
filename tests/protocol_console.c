/* Host-only protocol fixture. No physical hardware or persistent storage. */
#include "protocol.h"
#include <stdio.h>
#include <string.h>

static bool simulated_save(void *context, const uint8_t *data, size_t length)
{
    (void)context;
    EasyTVCSettings settings;
    return EasyTVC_SettingsDecode(&settings, data, length);
}

int main(int argc, char **argv)
{
    EasyTVCProtocol protocol;
    EasyTVC_ProtocolInit(&protocol);
    protocol.hardware_verified = argc == 2 && !strcmp(argv[1], "--verified-fixture");
    protocol.save = simulated_save;
    char response[EASYTVC_PROTOCOL_CAPACITY];
    int byte;
    while ((byte = getchar()) != EOF) {
        size_t size = EasyTVC_ProtocolFeed(&protocol, (char)byte, response);
        if (size) { fwrite(response, 1, size, stdout); fflush(stdout); }
    }
    return 0;
}
