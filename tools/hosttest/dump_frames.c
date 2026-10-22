#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "link_proto.h"
int main(void){
    uint8_t buf[300];
    uint8_t pl[5]={1,2,3,4,5};
    int n,i;
    n=Link_BuildFrame(buf,sizeof(buf),0x01,0x1234,0x06,pl,5);
    printf("roundtrip_len=%d hex=",n);
    for(i=0;i<n;i++) printf("%02X",buf[i]);
    printf("\n");
    n=Link_BuildFrame(buf,sizeof(buf),0x01,7,0x06,NULL,0);
    printf("ping_len=%d hex=",n);
    for(i=0;i<n;i++) printf("%02X",buf[i]);
    printf("\n");
    n=Link_BuildFrame(buf,sizeof(buf),0x01,0,0x08,pl,4);
    printf("getcfg_len=%d hex=",n);
    for(i=0;i<n;i++) printf("%02X",buf[i]);
    printf("\n");
    return 0;
}
