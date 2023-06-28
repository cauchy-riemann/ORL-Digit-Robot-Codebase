#include <qpOASES.hpp>
#include <iostream>
#include <limits>
USING_NAMESPACE_QPOASES

int main()
{
	real_t H[2*2] = { 3.0, 0.0,
		              0.0, 1.0 };
	real_t A[1*2] = { 1.0, 1.0 };
	real_t g[2] = { 1.5, 1.0 };
	real_t lb[2] = { 0.5, -2.0 };
	real_t ub[2] = { 5.0, 2.0 };
	real_t lbA[1] = { -1.0 };
	real_t ubA[1] = { 2.0 };

	/* Setting up SQProblem object. */
	SQProblem example( 2,1 );

	/* Solve first QP. */
	int_t nWSR = 10;
	real_t xOpt[2];
	example.init( H,g,A,lb,ub,lbA,ubA, nWSR,0 );
	example.getPrimalSolution( xOpt );
	std::cout << "QP Status: " << example.getStatus() << std::endl;
	printf( "\nxOpt = [ %e, %e ];  objVal = %e\n\n", xOpt[0],xOpt[1],example.getObjVal() );	
	return 0;
}



