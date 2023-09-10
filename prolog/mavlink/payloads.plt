:- begin_tests(mavlink_payloads).
:- use_module(payloads).

test(trailing_zeros, [true(A==3)]) :-
    mavlink_payloads:trailing_zeros([0, 0, 0], 0, A).
test(trailing_zeros, [true(A==0)]) :-
    mavlink_payloads:trailing_zeros([0, 0, 0, 1], 0, A).

:- end_tests(mavlink_payloads).
