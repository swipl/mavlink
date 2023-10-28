:- begin_tests(mavlink_frames).
:- use_module(frames).

test(mavlink_frame, true(A == 33)) :-
    phrase(mavlink_frame(A, _, _), [253,27,0,0,124,1,1,33,0,0,35,1,93,121,78,82,64,28,61,244,23,5,248,120,7,0,189,6,0,0,255,255,2,0,0,0,24,197,89]).

:- end_tests(mavlink_frames).
