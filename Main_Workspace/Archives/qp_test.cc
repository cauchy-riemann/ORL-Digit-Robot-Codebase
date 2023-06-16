#include <qpOASES.hpp>
#include <iostream>
#include <limits>
USING_NAMESPACE_QPOASES

int main()
{
	real_t H[2*2] = { 1.0, 0.0, 0.0, 0.5 };
	real_t A[1*2] = { 1.0, 1.0 };
	real_t g[2] = { 1.5, 1.0 };
	real_t lb[2] = { 0.5, -2.0 };
	real_t ub[2] = { 5.0, 2.0 };
	real_t lbA[1] = { -1.0 };
	real_t ubA[1] = { 2.0 };

	/* Setup data of second QP. */
	real_t H_new[2*2] = { 1.0, 0.5, 0.5, 0.5 };
	real_t A_new[1*2] = { 1.0, 5.0 };
	real_t g_new[2] = { 1.0, 1.5 };
	real_t lb_new[2] = { 0.0, -1.0 };
	real_t ub_new[2] = { 5.0, -0.5 };
	real_t lbA_new[1] = { -2.0 };
	real_t ubA_new[1] = { 1.0 };


	/* Setting up SQProblem object. */
	SQProblem example( 2,1 );

	/* Solve first QP. */
	int_t nWSR = 10;
	real_t xOpt[2];
	example.init( H,g,A,lb,ub,lbA,ubA, nWSR,0 );
	example.getPrimalSolution( xOpt );
	printf( "\nxOpt = [ %e, %e ];  objVal = %e\n\n", xOpt[0],xOpt[1],example.getObjVal() );
	
	/* Solve second QP. */
	nWSR = 10;
	example.hotstart( H_new,g_new,A_new,lb_new,ub_new,lbA_new,ubA_new, nWSR,0 );

	/* Get and print solution of second QP. */
	example.getPrimalSolution( xOpt );
	printf( "\nxOpt = [ %e, %e ];  objVal = %e\n\n", xOpt[0],xOpt[1],example.getObjVal() );
	
	return 0;
}



