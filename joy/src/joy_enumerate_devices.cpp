/*
 * Copyright (c) 2020, Open Source Robotics Foundation.
 * Copyright (c) 2023, CSIRO Data61.
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
 *     * Neither the name of the copyright holder nor the names of its
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

// On Windows, we have to be sure that SDL doesn't generate its own main.
#define SDL_MAIN_HANDLED
#include <SDL.h>

#include <cstdio>
#include <stdexcept>

int main()
{
  // SDL_INIT_GAMECONTROLLER implies SDL_INIT_JOYSTICK
  if (SDL_Init(SDL_INIT_GAMECONTROLLER | SDL_INIT_HAPTIC) < 0) {
    fprintf(stderr, "SDL could not be initialized: %s\n", SDL_GetError());
    return 1;
  }

  fprintf(
    stdout,
    "ID : GUID                             : GamePad : Mapped : Joystick Device Name\n");
  fprintf(
    stdout,
    "-------------------------------------------------------------------------------\n");
  for (int i = 0; i < SDL_NumJoysticks(); ++i) {
    SDL_JoystickGUID joystick_guid;
    const char * joystick_name = "Unknown";
    const char * has_mapping_string = "false";
    const char * is_gamepad = "false";

    if (SDL_IsGameController(i)) {
      SDL_GameController * controller = SDL_GameControllerOpen(i);
      joystick_guid = SDL_JoystickGetGUID(SDL_GameControllerGetJoystick(controller));
      joystick_name = SDL_GameControllerName(controller);
      if (nullptr != SDL_GameControllerMapping(controller)) {
        has_mapping_string = "true";
      }
      is_gamepad = "true";

    } else {
      joystick_name = SDL_JoystickNameForIndex(i);
      joystick_guid = SDL_JoystickGetDeviceGUID(i);
    }

    char guid_string[33];
    SDL_JoystickGetGUIDString(joystick_guid, guid_string, sizeof(guid_string));

    fprintf(
      stdout, "%2d : %32s : %7s : %6s : %s\n", i, guid_string, is_gamepad, has_mapping_string,
      joystick_name);
  }

  SDL_Quit();

  return 0;
}
