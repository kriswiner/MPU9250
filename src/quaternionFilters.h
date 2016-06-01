#ifndef _QUATERNIONFILTERS_H_
#define _QUATERNIONFILTERS_H_

void MadgwickQuaternionUpdate(float, float, float, float, float, float, float,
                              float, float);
void MahonyQuaternionUpdate(float, float, float, float, float, float, float,
                            float, float);

#endif // _QUATERNIONFILTERS_H_
