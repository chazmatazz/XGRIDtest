// sonar.h
#ifndef __SONAR_H__
#define __SONAR_H__

// currently the sonar is set up to output range in cm
// range extends to 6 meters, minimum is about 10 cm
// sonar range is always positive, unsigned int

void init_sonar(void);

uint16_t get_sonar_value(void); 

bool test_sonar_attached(void);

#endif