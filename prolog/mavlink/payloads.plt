:- begin_tests(mavlink_payloads).
:- use_module(payloads).

test(mavlink_payload, [true(A==[0, 0, 0, 64, 0, 0, 128, 63])]) :-
    mavlink_payload(173, [distance(2.0), voltage(1.0)], A, []).

test(trailing_zeros, [true(A==3)]) :-
    mavlink_payloads:trailing_zeros([0, 0, 0], 0, A).
test(trailing_zeros, [true(A==0)]) :-
    mavlink_payloads:trailing_zeros([0, 0, 0, 1], 0, A).

:- end_tests(mavlink_payloads).
