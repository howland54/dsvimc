/*=============================================================================
  Copyright (C) 2012 Allied Vision Technologies.  All Rights Reserved.

  Redistribution of this file, in original or modified form, without
  prior written consent of Allied Vision Technologies is prohibited.

-------------------------------------------------------------------------------

  File:        CameraObserver.cpp

  Description: A notification whenever device list has been changed


-------------------------------------------------------------------------------

  THIS SOFTWARE IS PROVIDED BY THE AUTHOR "AS IS" AND ANY EXPRESS OR IMPLIED
  WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF TITLE,
  NON-INFRINGEMENT, MERCHANTABILITY AND FITNESS FOR A PARTICULAR  PURPOSE ARE
  DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT,
  INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
  AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
  TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

=============================================================================*/
#include <iostream>
#include <string.h>

#include "CameraObserver.h"

CameraObserver::CameraObserver (  )
{

}

CameraObserver::~CameraObserver ( void )
{

}

void CameraObserver::CameraListChanged ( AVT::VmbAPI::CameraPtr pCam, AVT::VmbAPI::UpdateTriggerType reason )
{
   std::string theCameraSerialNumber;
   pCam->GetSerialNumber(theCameraSerialNumber);
  // char *serialNumberString = theCameraSerialNumber.c_str();

   switch (reason)
      {
      case AVT::VmbAPI::UpdateTriggerPluggedIn:
         {

            std::cout<<"camera sn: " << theCameraSerialNumber << " pluggedIn\n";
            msg_send(BUS_THREAD, BUS_THREAD, CAMA, strlen(theCameraSerialNumber.c_str()),theCameraSerialNumber.c_str());

            break;
         }
      case AVT::VmbAPI::UpdateTriggerPluggedOut:
         {
            std::cout<<"camera sn: " << theCameraSerialNumber << " pluggedOut\n";
            msg_send(BUS_THREAD, BUS_THREAD, CAMR, strlen(theCameraSerialNumber.c_str()),theCameraSerialNumber.c_str());

            break;
         }
      case AVT::VmbAPI::UpdateTriggerOpenStateChanged:
         {
            std::cout<<"camera sn: " << theCameraSerialNumber << " state changed!\n";

            break;
         }

      default:
         {
            printf("unknown camera event\n");
         }

      }
}
