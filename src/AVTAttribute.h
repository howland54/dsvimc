#ifndef AVTATTRIBUTE_H
#define AVTATTRIBUTE_H
#include <VimbaCPP/Include/VimbaCPP.h>
#include <iostream>
#include <sstream>

class AVTAttribute
{
public:
   AVTAttribute();
   VmbErrorType  setAttribute(const AVT::VmbAPI::CameraPtr &camera,std::string attributeName,std::string attributeValue);
private:
   AVT::VmbAPI::FeaturePtr      pFeature;

};

#endif // AVTATTRIBUTE_H
