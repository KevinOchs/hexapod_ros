/*
 * Copyright (c) 2009, Tully Foote
 * Copyright (c) 2011, Ivan Dryanovski
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef PHIDGETS_API_PHIDGET_H
#define PHIDGETS_API_PHIDGET_H

#include <phidgets_api/phidget21.h>

#include <iostream>
#include <algorithm>
#include <string>
#include <cstring>
#include <stdio.h>

namespace phidgets {

class Phidget
{
  public:

    Phidget();
    ~Phidget();

    /**@brief Open a connection to a Phidget
     * @param serial_number THe serial number of the phidget to which to attach (-1 will connect to any)*/
    int open(int serial_number);
    
    /**@brief Close the connection to the phidget */
    int close();
    
    /** @brief Block until the unit is attached or timeout occurs
     * @param timeout Milliseconds to wait before timing out */
    int waitForAttachment(int timeout);

    /** @brief Get the device type string */
    std::string getDeviceType();

    /** @brief Get the device name string */
    std::string getDeviceName();

    /** @brief Get the device label string */
    std::string getDeviceLabel();

    /** @brief Get the library version string */ 
    std::string getLibraryVersion();

    /** @brief Get the Phidget's serial number */
    int getDeviceSerialNumber();
    
    /** @brief Get the Phidget's version */
    int getDeviceVersion();
    
    /** @brief Lookup the string for a CPhidget Error Code 
     *@param errorCode The error code returned from the CPhidget API */
    static std::string getErrorDescription(int errorCode);
    
  protected:

    CPhidgetHandle handle_;
    
    void init(CPhidgetHandle handle);

    virtual void registerHandlers();
    virtual void attachHandler();
    virtual void detachHandler();
    virtual void errorHandler(int error);

  private:

    static int AttachHandler(CPhidgetHandle handle, void *userptr);
    static int DetachHandler(CPhidgetHandle handle, void *userptr);
    static int ErrorHandler (CPhidgetHandle handle, void *userptr, int ErrorCode, const char *unknown);
};

} // namespace phidgets

#endif // PHIDGETS_API_PHIDGET_H

