#include <emc/io.h>

#include <emc/io.h>
#include <unistd.h>
 
int main()
{
   emc::IO io; 
   while( io.ok() )
   {
      sleep(1);
      io.speak("test " );
   }
      
   return 0;
}

