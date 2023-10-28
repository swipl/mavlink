:- begin_tests(mavlink_grammars).
:- use_module(grammars).

test(array, [true(A==int(100))]) :-
    phrase(mavlink_array(A), `int[100]`).

test(basic, [true(A==`uint8_t`)]) :-
    phrase(mavlink_basic(uint8_t), A).

:- end_tests(mavlink_grammars).
