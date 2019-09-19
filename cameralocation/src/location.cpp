#include <cameralocation/location.h>


Location::Location(int casheSec)
{

    imuodom=Imuodom(120*casheSec); 
    
}

Location::~Location()
{
}



