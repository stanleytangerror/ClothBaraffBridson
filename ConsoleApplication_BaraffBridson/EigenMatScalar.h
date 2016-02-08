#ifndef EIGEN_MAT_SCALAR
#define EIGEN_MAT_SCALAR

#include <Eigen\Core>

/* according to http://eigen.tuxfamily.org/dox/structEigen_1_1NumTraits.html */
namespace Eigen {
	template<> struct NumTraits<Eigen::Matrix3f>
		: NumTraits<double> // permits to get the epsilon, dummy_precision, lowest, highest functions
	{
		/* giving the "real part" type of T*/
		typedef Eigen::Matrix3f Real;
		/* giving the type that should be used for operations producing non-integral values, 
		 such as quotients, square roots, etc. */
		typedef Eigen::Matrix3f NonInteger;
		/* A typedef Nested giving the type to use to nest a value inside of the expression tree. 
		If you don't know what this means, just use T here. */
		typedef Eigen::Matrix3f Nested;
		enum {
			IsComplex = 0,
			IsInteger = 0,
			IsSigned = 1,
			RequireInitialization = 1,
			ReadCost = 1,
			AddCost = 3,
			MulCost = 3
		};
	};
}
namespace adtl {
	inline const Eigen::Matrix3f& conj(const Eigen::Matrix3f& x) { return x.conjugate(); }
	inline const Eigen::Matrix3f& real(const Eigen::Matrix3f& x) { return x.real(); }
	inline Eigen::Matrix3f imag(const Eigen::Matrix3f& x) { return x.imag(); }
	inline Eigen::Matrix3f abs(const Eigen::Matrix3f&  x) { return Eigen::Matrix3f::Zero(); }
	inline Eigen::Matrix3f abs2(const Eigen::Matrix3f& x) { return Eigen::Matrix3f::Zero(); }
}

#endif
