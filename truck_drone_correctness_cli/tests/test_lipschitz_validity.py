#!/usr/bin/env python3
"""Exhaustively verify valid subset inequalities over small scheduling cases.

This is an independent mathematics check; it does not run Gurobi/OR-Tools.
"""
import itertools
import math
import random
from functools import lru_cache


def test_case(r, e, B, tau, K):
    n=len(r)
    @lru_cache(None)
    def bins(mask):
        if mask == 0: return 0
        # Pack one feasible cycle containing the least-index remaining item.
        low=mask & (-mask)
        others=mask ^ low
        b=math.inf
        sub=others
        while True:
            cycle=sub | low
            if sum(e[i] for i in range(n) if cycle>>i&1) <= B:
                b=min(b,1+bins(mask^cycle))
            if sub==0: break
            sub=(sub-1)&others
        return b

    @lru_cache(None)
    def one(mask):
        if mask==0: return 0
        return sum(r[i] for i in range(n) if mask>>i&1) + tau*(bins(mask)-1)

    @lru_cache(None)
    def multi(mask,k):
        if mask==0:return 0
        if k==1:return one(mask)
        best=one(mask)
        sub=mask
        while sub:
            best=min(best,max(one(sub),multi(mask^sub,k-1)))
            sub=(sub-1)&mask
        return best

    count=0
    for S in range(1<<n):
        for D in range(1<<n):
            # Cut is valid for ALL assignments D (including added customers),
            # with a certified L(S) <= true optimum for S.
            for k in (1,K):
                optimum_S=multi(S,k)
                # Check two certified bound choices, including a possibly weak LB.
                for L in {optimum_S, optimum_S//2}:
                    delta=sum(min(r[i]+tau,L) for i in range(n)
                              if S>>i&1 and not D>>i&1)
                    assert multi(D,k)+delta>=L, (r,e,B,tau,k,S,D,L)
                    count+=1
            # Aggregate workload inequality from battery count energy LB.
            R=max(0,math.ceil(sum(e[i] for i in range(n) if S>>i&1)/B)-K)
            F=sum(r[i] for i in range(n) if S>>i&1)+tau*R
            delta=sum(min(r[i]+tau,F) for i in range(n)
                      if S>>i&1 and not D>>i&1)
            assert K*multi(D,K)+delta>=F, (r,e,B,tau,K,S,D)
            q=max(0,bins(D)-K)
            missing=(S & ~D).bit_count()
            assert q+missing>=R, (e,B,K,S,D,q,R)
            count+=2
            # One-drone battery cycle min swaps cut.
            R1=max(0,math.ceil(sum(e[i] for i in range(n) if S>>i&1)/B)-1)
            assert max(0,bins(D)-1)+missing>=R1
            count+=1
    return count


def main():
    random.seed(20261107)
    cases=0; checks=0
    for n in (1,2,3,4,5,6):
        for _ in range(4):
            B=random.randint(4,16)
            e=[random.randint(1,B) for _ in range(n)]
            r=[random.randint(0,20) for _ in range(n)]
            tau=random.randint(0,8)
            K=random.randint(1,4)
            checks+=test_case(r,e,B,tau,K)
            cases+=1
    print(f'PASS: {checks:,} valid-inequality checks over {cases} parameter settings')
if __name__=='__main__':main()
