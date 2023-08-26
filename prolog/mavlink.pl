/*  File:    mavlink.pl
    Author:  Roy Ratcliffe
    Created: Aug 20 2023
    Purpose: MAVLink
*/

:- module(mavlink,
          []).

/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

mavlink_definitions_r(all, A),
    forall((   member(B-C, A),
               mavlink_definitions:mavlink_definition(C, D)),
           assertz(mavlink:D)).

- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */
