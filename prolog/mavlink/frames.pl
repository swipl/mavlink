/*  File:    mavlink/frames.pl
    Author:  Roy Ratcliffe
    Created: Aug 27 2023
    Purpose: MAVLink Frames
*/

:- module(mavlink_frames,
          [ mavlink_frame//3
          ]).
:- use_module(library(mavlink/crc_16_mcrf4xx)).
:- use_module(library(mavlink/extras)).
:- use_module(endian).

/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    Accumulates the CRC-16/MCRF4XX progressively.

- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */

%!  mavlink_frame(?Msg, ?Payload, -Attrs)// is semidet.
%
%   @arg Attrs is a list of frame attribute terms including the version
%   number, frame length, sequence number, system and component
%   identifiers. You *cannot* use the argument to pre-filter frames
%   matching particular required attributes; post-filter if necessary
%   by decoding the frame first then select on attributes.

mavlink_frame(Msg, Payload,
              [ ver(1),
                len(Len),
                seq(Seq),
                sys(Sys),
                comp(Comp)
              ]) -->
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
mavlink_frame(Msg, Payload,
              [ ver(2),
                len(Len),
                incompat(Incompat),
                compat(Compat),
                seq(Seq),
                sys(Sys),
                comp(Comp)
              ]) -->
    [STX],
    { ver_stx(2, STX),
      crc_16_mcrf4xx(Check0)
    },
    [Len],
    { crc_16_mcrf4xx(Check0, Len, Check1)
    },
    [Incompat],
    { crc_16_mcrf4xx(Check1, Incompat, Check1A)
    },
    [Compat],
    { crc_16_mcrf4xx(Check1A, Compat, Check1B)
    },
    [Seq],
    { crc_16_mcrf4xx(Check1B, Seq, Check2)
    },
    [Sys],
    { crc_16_mcrf4xx(Check2, Sys, Check3)
    },
    [Comp],
    { crc_16_mcrf4xx(Check3, Comp, Check4)
    },
    endian(little, 24, Msg),
    { Msg0 is Msg /\ 16'FF,
      Msg1 is (Msg >> 8) /\ 16'FF,
      Msg2 is Msg >> 16,
      crc_16_mcrf4xx(Check4, [Msg0, Msg1, Msg2], Check5),
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
