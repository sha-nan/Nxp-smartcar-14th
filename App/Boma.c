#include "Boma.h"
#include "common.h"
#include "Public.h"

int k1,k2,k3,k4;
int BOMA=0;
//≤¶¬Î≈Ã≥ı ºªØ
void  Boma_init()
{
   gpio_init(PTC8,GPI,0);
   port_init_NoALT (PTC8,  PF | PULLDOWN );
   gpio_init(PTC9,GPI,0);
   port_init_NoALT (PTC9,  PF | PULLDOWN );
   gpio_init(PTC10,GPI,0);
   port_init_NoALT (PTC10,  PF | PULLDOWN );
   gpio_init(PTC11,GPI,0);
   port_init_NoALT (PTC11,  PF | PULLDOWN );

}

//≤¶¬Î≈Ã∂¡»°
void Boma_read()
{
   if(PTC8_IN == 1)   k4=1;
   else k4=0;
   if(PTC9_IN == 1)   k3=1;
   else k3=0;
   if(PTC10_IN == 1)  k2=1;
   else k2=0;
   if(PTC11_IN == 1)  k1=1;
   else k1=0;

   BOMA=1*k1+2*k2+4*k3+8*k4;
}

