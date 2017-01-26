/*
 * Copyright (c) 2007, Vanderbilt University
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * - Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * - Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the
 *   distribution.
 * - Neither the name of the copyright holder nor the names of
 *   its contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL
 * THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Miklos Maroti
 */

#define stk_size 3000

module AssertP
{
	uses
	{
		interface DiagMsg;
		interface Leds;
	}
}

implementation
{

	char *stk_prt[stk_size];
	uint16_t stk_ind = 0;
	uint16_t prt_ind = 0;
	uint8_t print_enable = 1;

	task void send() {
		if (print_enable) {
			call Leds.led7On();
		} else call Leds.led6On();
		// PRT_T("sending");
		if (prt_ind < stk_size) {
			while (prt_t(stk_prt[prt_ind]) && ++prt_ind < stk_size);
			if (prt_ind < stk_size) {
				post send();
			} else {
				// PRT_T("no longer inner");
			}
		} else {
			// PRT_T("no longer");
		}
	}

	void f_stk_prt (char *inp) __attribute__((noinline)) @C() @spontaneous() {
		atomic {
			if (print_enable) {
				stk_prt[stk_ind++] = inp;
				if (stk_ind>=stk_size) {
					print_enable = 0;
					post send();
					stk_ind = 0;
					print_enable = 0;
				}
			}
		}
	}

	bool prt_t(const char* inp) __attribute__((noinline)) @C() @spontaneous() {
		// while (! call DiagMsg.record());
		atomic {
			if( call DiagMsg.record() ) {
				call DiagMsg.str(inp);
				call DiagMsg.send();
				return TRUE;
			}
			return FALSE;
		}
	}

	void assert(bool condition, const char* file, uint16_t line) __attribute__((noinline)) @C() @spontaneous()
	{
		if( ! condition )
		{
#ifdef ASSERT_LEDON
			call Leds.led4Toggle();
#endif

			if( call DiagMsg.record() )
			{
				uint8_t del = 0;
				uint8_t len = 0;

				while( file[len] != 0 )
				{
					if( file[len] == '\\' || file[len] == '/' )
						del = len + 1;

					++len;
				}

				file += del;

				call DiagMsg.str("assert");
				call DiagMsg.str(file);
				call DiagMsg.uint16(line);
				call DiagMsg.send();
			}
		}
	}
}
