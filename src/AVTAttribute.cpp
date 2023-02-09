#include "AVTAttribute.h"

AVTAttribute::AVTAttribute()
{

}

VmbErrorType  AVTAttribute::setAttribute(const AVT::VmbAPI::CameraPtr &camera,std::string attributeName,std::string attributeValue)
{
   if(!camera)
      {
         return VmbErrorOther;
      }

   VmbErrorType res = SP_ACCESS( camera )->GetFeatureByName( attributeName.data(), pFeature );
   VmbFeatureDataType  featureType;
   if( VmbErrorSuccess == res )
      {
         res = pFeature->GetDataType( featureType );
      }
   if( VmbErrorSuccess == res )
      {
         switch( featureType )
            {
            bool ok;
            case VmbFeatureDataBool:
               {
                  int theValue;
                  try
                  {
                     theValue = std::stoi(attributeValue);
                  }
                  catch(...)
                  {
                     std::stringstream ss;
                     ss.str( "" );
                     ss << "error in loading " << attributeName ;
                     std::cout << ss.str() << std::endl;
                     return VmbErrorOther;
                  }
                  VmbBool_t value = (VmbBool_t)theValue;

                  res = pFeature->SetValue(value);
                  return res;

               }
               break;
            case VmbFeatureDataEnum:
            case VmbFeatureDataString:
               {
                  char *value = (char *)attributeValue.data();
                  res = pFeature->SetValue(value);
                  return res;
               }
               break;
            case VmbFeatureDataFloat:
               {
                  double theValue;
                  try
                  {
                     theValue = std::stod(attributeValue);
                  }
                  catch(...)
                  {
                     std::stringstream ss;
                     ss.str( "" );
                     ss << "error in loading " << attributeName ;
                     std::cout << ss.str() << std::endl;
                     return VmbErrorOther;
                  }
                  double value = (double)theValue;

                  res = pFeature->SetValue(value);
                  return res;

               }
               break;
            case VmbFeatureDataInt:
               {
                  VmbInt64_t theValue;
                  try
                  {
                     theValue = std::stoi(attributeValue);
                  }
                  catch(...)
                  {
                     std::stringstream ss;
                     ss.str( "" );
                     ss << "error in loading " << attributeName ;
                     std::cout << ss.str() << std::endl;
                     return VmbErrorOther;
                  }
                  VmbInt64_t value = (VmbInt64_t)theValue;

                  res = pFeature->SetValue(value);
                  return res;

               }
               break;
            default:
               res = VmbErrorInvalidCall;
               break;
            }
      }


   return res;

}


