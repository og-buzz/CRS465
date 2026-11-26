
;;
;; Recursion / Call tests
;;
;; 	$Id: fib.r3 1.1 1997/02/14 18:35:19 dcm Exp $
;;


func int fib( int n )
  ;; Compute the nth Fibonacci number recursively...

  if n <= 2
    return 1
  end if

  return fib(n - 1) + fib( n - 2)
end func


main

  print("Recursive Fibonacci Number Calculation:\n")

  int i, n, last = 1

  for i = 1 to 24
    n = fib(i)
    printf("fib({2}) = {5} (ratio = {9.8})\n", i, n, <float>n / <float>last)
    last = n
  end for

end main

