// ***** BEGIN LICENSE BLOCK *****
//
// Copyright (c) 2006-2008, NIF File Format Library and Tools.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above
//      copyright notice, this list of conditions and the following
//      disclaimer in the documentation and/or other materials provided
//      with the distribution.
//
//    * Neither the name of the NIF File Format Library and Tools
//      project nor the names of its contributors may be used to endorse
//      or promote products derived from this software without specific
//      prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// ***** END LICENSE BLOCK *****

#include "NifMopp.h"


#if _MSC_VER >= 1900
#pragma comment(lib, "legacy_stdio_definitions.lib")

FILE _iob[] = { *stdin, *stdout, *stderr };

extern "C" FILE * __iob_func( void )
{
	return _iob;
}
#endif


//
// Math and base include
#include <Common/Base/hkBase.h>
#include <Common/Base/System/hkBaseSystem.h>
#include <Common/Base/Memory/hkThreadMemory.h>
#include <Common/Base/Memory/Memory/Pool/hkPoolMemory.h>
#include <Common/Base/System/Error/hkDefaultError.h>
#include <Common/Base/Monitor/hkMonitorStream.h>

#pragma comment(lib, "hkBase.lib")
#pragma comment(lib, "hkSerialize.lib")
#pragma comment(lib, "hkInternal.lib")
#pragma comment(lib, "hkpUtilities.lib")

#ifdef _MANAGED
#pragma managed(push, off)
#endif

// External references
extern void CloseHavokMopp();


static void HK_CALL errorReport(const char* msg, void*)
{
	//printf("%s", msg);
}

static hkThreadMemory* threadMemory = NULL;
static char* stackBuffer = NULL;
extern void InitializeHavok()
{
	if ( threadMemory == NULL )
	{
		// Initialize the base system including our memory system
		hkPoolMemory* memoryManager = new hkPoolMemory();
		threadMemory = new hkThreadMemory(memoryManager);
		hkBaseSystem::init( memoryManager, threadMemory, errorReport );
		memoryManager->removeReference();

		// We now initialize the stack area to 100k (fast temporary memory to be used by the engine).
		{
			int stackSize = 0x100000;
			stackBuffer = hkAllocate<char>( stackSize, HK_MEMORY_CLASS_BASE);
			hkThreadMemory::getInstance().setStackArea( stackBuffer, stackSize);
		}
	}
}

static void CloseHavok()
{
	CloseHavokMopp();

	// Deallocate stack area
	if (threadMemory)
	{
		threadMemory->setStackArea(0, 0);
		hkDeallocate(stackBuffer);

		threadMemory->removeReference();
		threadMemory = NULL;
		stackBuffer = NULL;
	}

	// Quit base system
	hkBaseSystem::quit();
}

BOOL APIENTRY DllMain( HMODULE hModule,
                       DWORD  ul_reason_for_call,
                       LPVOID lpReserved
					 )
{
	if ( ul_reason_for_call == DLL_PROCESS_DETACH )
		CloseHavok();
    return TRUE;
}


#ifdef _MANAGED
#pragma managed(pop)
#endif

