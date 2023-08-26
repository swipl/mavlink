/*  File:    mavlink/crc_16_mcrf4xx.pl
    Author:  Roy Ratcliffe
    Created: Aug 20 2023
    Purpose: MAVLink CRC-16-MCRF4XX

Copyright (c) 2023, Roy Ratcliffe, Northumberland, United Kingdom

Permission is hereby granted, free of charge,  to any person obtaining a
copy  of  this  software  and    associated   documentation  files  (the
"Software"), to deal in  the   Software  without  restriction, including
without limitation the rights to  use,   copy,  modify,  merge, publish,
distribute, sublicense, and/or sell  copies  of   the  Software,  and to
permit persons to whom the Software is   furnished  to do so, subject to
the following conditions:

    The above copyright notice and this permission notice shall be
    included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT  WARRANTY OF ANY KIND, EXPRESS
OR  IMPLIED,  INCLUDING  BUT  NOT   LIMITED    TO   THE   WARRANTIES  OF
MERCHANTABILITY, FITNESS FOR A PARTICULAR   PURPOSE AND NONINFRINGEMENT.
IN NO EVENT SHALL THE AUTHORS  OR   COPYRIGHT  HOLDERS BE LIABLE FOR ANY
CLAIM, DAMAGES OR OTHER LIABILITY,  WHETHER   IN  AN ACTION OF CONTRACT,
TORT OR OTHERWISE, ARISING FROM,  OUT  OF   OR  IN  CONNECTION  WITH THE
SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

*/

:- module(mavlink_crc_16_mcrf4xx,
          [ crc_16_mcrf4xx/1,                 % -Check
            crc_16_mcrf4xx/3                  % +Check0,+Data,-Check
          ]).

%!  crc_16_mcrf4xx(-Check) is det.
%
%   Initialises CRC-16/MCRF4XX checksum.

crc_16_mcrf4xx(16'FFFF).

%!  crc_16_mcrf4xx(+Check0, +Data, -Check) is det.
%
%   Accumulates CRC-16/MCRF4XX checksum using optimal shifting and
%   exclusive-OR operations.

crc_16_mcrf4xx(Check0, Data, Check) :-
    integer(Data),
    !,
    Data_ is (Check0 /\ 16'FF) xor (Data /\ 16'FF),
    Data__ is (Data_ xor (Data_ << 4)) /\ 16'FF,
    Check is (Check0 >> 8) xor (Data__ << 8) xor (Data__ << 3) xor (Data__ >> 4).
crc_16_mcrf4xx(Check0, Data, Check) :-
    is_list(Data),
    !,
    foldl(crc_16_mcrf4xx_, Data, Check0, Check).
crc_16_mcrf4xx(Check0, Data, Check) :-
    atomic(Data),
    atom_codes(Data, Data_),
    crc_16_mcrf4xx(Check0, Data_, Check).

crc_16_mcrf4xx_(Data, Check0, Check) :- crc_16_mcrf4xx(Check0, Data, Check).
