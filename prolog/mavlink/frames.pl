/*  File:    mavlink/frames.pl
    Author:  Roy Ratcliffe
    Created: Aug 27 2023
    Purpose: MAVLink Frames
*/

:- module(mavlink_frames,
          [ mavlink_frame//1
          ]).
:- use_module(library(mavlink/crc_16_mcrf4xx)).
:- use_module(library(mavlink/extras)).

/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    Accumulates the CRC-16/MCRF4XX progressively.

- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */

%!  mavlink_frame(Frame)// is semidet.

mavlink_frame(mavlink(Msg, Payload,
                      [ ver(1),
                        len(Len),
                        seq(Seq),
                        sys(Sys),
                        comp(Comp)
                      ])) -->
    [STX],
    { ver_stx(1, STX),
      crc_16_mcrf4xx(Check0)
    },
    [Len],
    { crc_16_mcrf4xx(Check0, Len, Check1)
    },
    [Seq],
    { crc_16_mcrf4xx(Check1, Seq, Check2)
    },
    [Sys],
    { crc_16_mcrf4xx(Check2, Sys, Check3)
    },
    [Comp],
    { crc_16_mcrf4xx(Check3, Comp, Check4)
    },
    [Msg],
    { crc_16_mcrf4xx(Check4, Msg, Check5),
      length(Payload, Len)
    },
    Payload,
    { crc_16_mcrf4xx(Check5, Payload, Check6),
      mavlink_extra(Msg, Extra),
      crc_16_mcrf4xx(Check6, Extra, Check7),
      CheckLo is Check7 /\ 16'FF
    },
    [CheckLo],
    { CheckHi is Check7 >> 8
    },
    [CheckHi].

ver_stx(1, 16'FE).
ver_stx(2, 16'FD).
