
;;
;; The Classic Sieve of Eratosthenes Benchmark
;;
;;	$Id: sieve.r3 1.1 1997/02/14 18:35:34 dcm Exp $
;;


.define size   8190
.define ntimes 25
.define TRUE   1
.define FALSE  0


int[size + 1] flag


main
  int  i
  int  j
  int count
  int prime
  int k

  print("\n\n", ntimes, " iterations.\n")

  for i = 1 to ntimes
    count = 0
    for k = 0 to size
      flag[k] = TRUE
    end for

    for j = 0 to size
      if flag[j] then
	prime = (j << 1) + 3
	k = j + prime
	while k <= size
	  flag[k] = FALSE
	  k += prime
	end while
	count++
      end if
    end for
  end for
  print("\n", count, " primes found.\n\n")
end main
