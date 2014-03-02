#TSP

Solution to traveling salesman problem as specified by Codeeval
https://www.codeeval.com/open_challenges/90/

##Details

Implements both O(n!) and O(2^n*n^2) via dynamic programming and more space.

##Usage

###Input format

```
5 | Dropbox 185 Berry St, SF (37.7768800, -122.3911496)
4 | Airbnb 99 Rhode Island St, SF (37.7689269, -122.4029053)
6 | Zynga 699 8th St, SF (37.7706628, -122.4040139)
```

Default runtime

```
$ time ./tsp tests/test_546.txt
5
4
6

real	0m0.003s
user	0m0.000s
sys	0m0.000s
```

Slower version, remember add more items to see the difference.

```
$ time ./tsp tests/test_546.txt --slow
5
4
6

real	0m0.003s
user	0m0.004s
sys	0m0.000s
```
