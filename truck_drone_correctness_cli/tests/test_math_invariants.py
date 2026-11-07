#!/usr/bin/env python3
"""Small dependency-free checks of corrected mathematical corner cases.
These are NOT replacements for linked Gurobi/OR-Tools integration tests.
"""
import itertools
import unittest

class Invariants(unittest.TestCase):
    def test_sparse_assignment_drone_makespan_is_not_zero(self):
        # K=4, D={1,2}, each sortie is nonempty and each drone can do one.
        k=4
        times=[10,15]
        self.assertLess(len(times),k)
        self.assertEqual(max(times),15)
        self.assertGreater(max(times),0)

    def test_mtz_n_incorrect_with_unselected_customer(self):
        # A path with n=4 customers and last active ord_i=4;
        # unused j has ord_j=0; arc x_ij=0. For n=4:
        # ord_j >= ord_i+1 - n fails, while n+1 is tight and valid.
        n=4
        ord_i=4
        ord_j=0
        self.assertFalse(ord_j>=ord_i+1-n)
        self.assertTrue(ord_j>=ord_i+1-(n+1))

    def test_subset_benders_cut(self):
        # L(S)=9 for S={1,2}, monotonic valid lower bounds for other D.
        # If S is subset of D, the cut enforces zD>=9; otherwise RHS<=0.
        S={1,2}
        L=9
        for bits in itertools.product((0,1),repeat=3):
            D={i+1 for i,v in enumerate(bits) if v}
            actual=9 if S.issubset(D) else 0  # minimal abstract monotone function
            lhs=actual+L*len(S-D)
            self.assertGreaterEqual(lhs,L)

    def test_integer_deadline_under_fractional_ub(self):
        # Largest INTEGER makespan strictly less than UB.
        import math
        for ub,want in [(100.0,99),(100.5,100),(101,100)]:
            T=math.ceil(ub-1e-9)-1
            self.assertEqual(T,want)
            self.assertGreaterEqual(T+1,ub-1e-9)

if __name__=='__main__':unittest.main()
