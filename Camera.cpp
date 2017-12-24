// MIT License
//
// Copyright (c) 2017 Artem Zhuravsky
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#define XPLM200 = 1;

#include "XPLMCamera.h"
#include "XPLMDataAccess.h"
#include "XPLMDisplay.h"
#include "XPLMGraphics.h"
#include "XPLMMenus.h"
#include "XPLMPlugin.h"
#include "XPLMProcessing.h"
#include "XPLMScenery.h"
#include "XPLMUtilities.h"
#include "XPWidgets.h"
#include "XPStandardWidgets.h"

#include <netdb.h> 
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <thread>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>

using namespace std;

static XPLMHotKeyID	gHotKey = NULL; 

static float HeadUpdateFlightLoopCallback(
                           float                inElapsedSinceLastCall,
                           float                inElapsedTimeSinceLastFlightLoop,
                           int                  inCounter,
                           void *               inRefcon);

static void	MyHotKeyCallback(void *inRefcon);

static int 	MyCameraFunc(  XPLMCameraPosition_t * outCameraPosition,  
                           int                  inIsLosingControl,    
                           void *               inRefcon); 
 
static bool my_camera_engaged = false;

// TODO make atomic
static float head_offset = 0.0;
static float pitch_offset = 0.0;
static float pitch_offset_start = 0.0;

static unique_ptr<thread> tcp_client_thread;

// TODO make atomic
static bool client_run = false;


static XPLMDataRef  pilot_head_x_ref;
static XPLMDataRef  pilot_head_y_ref;
static XPLMDataRef  pilot_head_z_ref;
static XPLMDataRef  pilot_head_heading_ref;
static XPLMDataRef  pilot_head_pitch_ref;

void tcp_client_worker() {
  XPLMDebugString("Client thread started.");

  int sockfd, port_no;
  struct sockaddr_in serv_addr;
  struct hostent *server;

  sockfd = socket(AF_INET, SOCK_STREAM, 0);
  if (sockfd < 0) {
    XPLMDebugString("ERROR opening socket");
  }
  server = gethostbyname("localhost");
  port_no = 3001;

  // Connecting
  bzero((char *) &serv_addr, sizeof(serv_addr));
  serv_addr.sin_family = AF_INET;
  bcopy((char *)server->h_addr, 
       (char *)&serv_addr.sin_addr.s_addr,
       server->h_length);
  serv_addr.sin_port = htons(port_no);
  if (connect(sockfd, (struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0) {
    XPLMDebugString("ERROR connecting");
  }

  char buffer[100];
  while (client_run) {
    // Read size of block.
    int n = read(sockfd, buffer, 1);
    if (n <= 0) {
      XPLMDebugString("ERROR reading from socket");
    }

    // Read the block itself.
    n = read(sockfd, buffer, buffer[0]);
    if (n <= 0) {
      XPLMDebugString("ERROR reading from socket");
    }

    double heading = 0, pitch = 0;//z = 0;
    memcpy(&heading, buffer, sizeof heading);
    memcpy(&pitch, &(buffer[8]), sizeof pitch);

    float new_offset = (float)(2.0 * heading);
    if (new_offset > 90.0) new_offset = 90.0;
    if (new_offset < -90.0) new_offset = -90.0;
    head_offset = new_offset;

    float pitch_new_offset = pitch;
    if (pitch_new_offset > 70.0) new_offset = 70.0;
    if (pitch_new_offset < -70.0) new_offset = -70.0;
    // negate the value, 'cause for x-plane "moving up - positive pitch",
    // while for the server it's the opposite.
    pitch_offset = - pitch_new_offset;
  }

  close(sockfd);
  XPLMDebugString("Client thread finished.");
}

PLUGIN_API int XPluginStart( char *     outName,
                             char *     outSig,
                             char *     outDesc) {
  strcpy(outName, "Camera HT");
  strcpy(outSig, "AZhuravsky.Experimental.CameraHT");
  strcpy(outDesc, "DIY head-tracking.");
 
  pilot_head_x_ref = XPLMFindDataRef("sim/graphics/view/pilots_head_x");
  pilot_head_y_ref = XPLMFindDataRef("sim/graphics/view/pilots_head_y");
  pilot_head_z_ref = XPLMFindDataRef("sim/graphics/view/pilots_head_z");
  pilot_head_heading_ref = XPLMFindDataRef("sim/graphics/view/pilots_head_psi");
  pilot_head_pitch_ref = XPLMFindDataRef("sim/graphics/view/pilots_head_the");

  /* Register our hot key. */
  gHotKey = XPLMRegisterHotKey(XPLM_VK_F8, xplm_DownFlag, 
				"Engage my camera control",
				MyHotKeyCallback,
				NULL);
  return 1;
}
 
 
PLUGIN_API void	XPluginStop(void) {
  // Nothing to do, as for now.
}
 
 
PLUGIN_API void XPluginDisable(void) {
  XPLMUnregisterFlightLoopCallback(HeadUpdateFlightLoopCallback, 0);
}
 
 
PLUGIN_API int XPluginEnable(void) {
  XPLMRegisterFlightLoopCallback(HeadUpdateFlightLoopCallback, -1.0, 0);
  return 1;
}
 
 
PLUGIN_API void XPluginReceiveMessage(XPLMPluginID   inFromWho,
                                      long           inMessage,
                                      void *         inParam) {
}
 
float HeadUpdateFlightLoopCallback(
                           float                inElapsedSinceLastCall,    
                           float                inElapsedTimeSinceLastFlightLoop,    
                           int                  inCounter,    
                           void *               inRefcon) {
  if (my_camera_engaged) {
    float norm_head_offset = 0.0;
    if (head_offset < 0.0) {
      norm_head_offset = 360.0 + head_offset;
    } else {
      norm_head_offset = head_offset;
    }

    // TODO Set the values only when they have actually been changed.
    XPLMSetDataf(pilot_head_heading_ref, norm_head_offset);
    XPLMSetDataf(pilot_head_pitch_ref, pitch_offset_start + pitch_offset);

    // Call the callback again in 50ms.
    return 0.05;
  }
  // When it's disengaged, call the callback again 1 second later.
  return 1.0;
}

void MyHotKeyCallback(void *inRefcon) {
  if (!my_camera_engaged) {
    XPLMDebugString("Get camera control!");

    pitch_offset_start = XPLMGetDataf(pilot_head_pitch_ref);
    head_offset = 0.0;
    pitch_offset = 0.0;
    
    // Constructs and starts the new thread and runs it. Does not block execution.
    client_run = true;
    tcp_client_thread.reset(new thread(tcp_client_worker));
    tcp_client_thread->detach();
  } else {
    client_run = false;
    XPLMDebugString("Released camera control.");
  }
  my_camera_engaged = !my_camera_engaged;
}

