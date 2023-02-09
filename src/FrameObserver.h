/*=============================================================================
  Copyright (C) 2013 Allied Vision Technologies.  All Rights Reserved.

  Redistribution of this file, in original or modified form, without
  prior written consent of Allied Vision Technologies is prohibited.

-------------------------------------------------------------------------------

  File:        FrameObserver.h

  Description: The frame observer that is used for notifications from VimbaCPP
               regarding the arrival of a newly acquired frame.

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

#ifndef AVT_VMBAPI_EXAMPLES_FRAMEOBSERVER
#define AVT_VMBAPI_EXAMPLES_FRAMEOBSERVER

#include <lcm/lcm-cpp.hpp>

#include "VimbaCPP/Include/VimbaCPP.h"
#include <sys/timeb.h>
#include <sys/stat.h>

#include <../../dsvimlib/include/time_util.h>
#include <../../dsvimlib/include/mkdir_p.h>
#include <../../dsvimlib/include/msg_util.h>		/* utility functions for messaging */

#include <mesobot-lcmtypes/image/image_t.hpp>

#include "vimc.h"
extern lcm::LCM myLcm;
extern lcm::LCM squeezeLcm;

class FrameObserver : virtual public AVT::VmbAPI::IFrameObserver
{

public:
    // We pass the camera that will deliver the frames to the constructor
    FrameObserver(AVT::VmbAPI::CameraPtr pCamera, int theCameraNumber) ;
    
    // This is our callback routine that will be executed on every received frame
    virtual void FrameReceived( const AVT::VmbAPI::FramePtr pFrame );
    void                               setRegionString(std::string theRegionString);



private:


    //std::string theDataDir;
    char                                theDataDir[512];
    unsigned long                       imageCount;
    bool                                startTimeHasBeenSet;
    int                                 cameraNumber;

    template <typename T>
    class ValueWithState
    {
    private:
        T m_Value;
        bool                                 m_State;



    public:
        ValueWithState()
            : m_State( false )
        {}
        ValueWithState( T &value )
            : m_Value ( value )
            , m_State( true )
        {}
        const T& operator()() const
        {
            return m_Value;
        }
        void operator()( const T &value )
        {
            m_Value = value;
            m_State = true;
        }
        bool IsValid() const
        {
            return m_State;
        }
        void Invalidate()
        {
            m_State = false;
        }
    };
    bool                        m_interactive;

    ValueWithState<double>      m_FrameTime;
    ValueWithState<VmbUint64_t> m_FrameID;
    VmbPixelFormatType                   m_Format;
    VmbUint32_t                          m_nWidth;
    VmbUint32_t                          m_nHeight;
    VmbUint32_t                          m_nSize;

    int                         m_area_threshold;
    int                         m_intensityBias;
    double                      m_intensityGain;
    int                         m_roiPadding;
    double                      m_focus;

    AVT::VmbAPI::CameraPtr     myCameraPtr;
    std::string                regionString;

    int lastYear;
    int lastMonth;
    int lastDay;
    int lastHour;
    int lastMinute;
    int lastDayOfYear;





private:
};


#endif
