#include "board.h" 


uint32_t BoardGetRandomSeed(void) 
{
  return 4; 

  /*
  union 
  {
    uint16_t id16[2]; 
    uint32_t id32; 
  } id; 
  BoardGetUniqueId(&id.id16[0]);
  id.id16[1] = id.id16[0]; 
  return SysTick->VAL ^ id.id32 ;
  */

}
static int iid = -1; 

void BoardGetUniqueId( uint16_t *id )
{
  if (iid < 0) 
  {
    uint32_t val1, val2, val3, val4;
    uint32_t *ptr1 = (volatile uint32_t *)0x0080A00C;
    val1 = *ptr1;
    uint32_t *ptr = (volatile uint32_t *)0x0080A040;
    val2 = *ptr;
    ptr++;
    val3 = *ptr;
    ptr++;
    val4 = *ptr;
    iid = (val1 ^ val2 ^ val3 ^val4) >> 16;
  }
  *id = (uint16_t) iid;
}



Version_t BoardGetVersion( void )
{
  Version_t v = {.Value = 0}; 
  return v;
}
