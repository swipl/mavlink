/*  File:    mavlink_crc_16_mcrf4xx.pl
    Author:  Roy Ratcliffe
    Created: Aug 20 2023
    Purpose: MAVLink CRC-16-MCRF4XX
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
    Data__ is Data_ xor ((Data_ << 4) /\ 16'FF),
    Check is (Check0 >> 8) xor (Data__ << 8) xor (Data__ << 3) xor (Data__ >> 4).
crc_16_mcrf4xx(Check0, Data, Check) :-
    is_list(Data),
    foldl(crc_16_mcrf4xx_, Data, Check0, Check).

crc_16_mcrf4xx_(Data, Check0, Check) :- crc_16_mcrf4xx(Check0, Data, Check).
