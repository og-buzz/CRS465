;; small end-to-end test: a recursive function, a loop, ints and floats, a string
func int fib(int n)
  if n <= 2
    return 1
  end if
  return fib(n - 1) + fib(n - 2)
end func

main
  int i, total = 0
  float f
  for i = 1 to 10
    total = total + fib(i)
  end for
  f = <float>total / 4.0
  printf("sum of fib(1..10) = {}\n", total)
  printf("quarter = {}\n", f)
  print("done\n")
end main
