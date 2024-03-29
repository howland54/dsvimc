﻿/*=============================================================================
  Copyright (C) 2012 Allied Vision Technologies.  All Rights Reserved.

  Redistribution of this file, in original or modified form, without
  prior written consent of Allied Vision Technologies is prohibited.

-------------------------------------------------------------------------------

  File:        CameraObserver.h

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


#ifndef CAMERAOBSERVER_H
#define CAMERAOBSERVER_H

#include <VimbaCPP/Include/VimbaSystem.h>
#include <VimbaCPP/Include/ICameraListObserver.h>

#include "../../dsvimlib/include/convert.h"
#include "../../dsvimlib/include/global.h"
#include "../../dsvimlib/include/imageTalk.h"		/* jasontalk protocol and structures */
#include "../../dsvimlib/include/msg_util.h"		/* utility functions for messaging */


class CameraObserver : public AVT::VmbAPI::ICameraListObserver
{

    public:

        CameraObserver();
       ~CameraObserver(void);

        virtual void CameraListChanged( AVT::VmbAPI::CameraPtr pCam, AVT::VmbAPI::UpdateTriggerType reason );            

    private:



};

#endif
