#pragma once

class Functor
{
	public:
	Functor(){}
	
	virtual void accumulate()=0;
	
	typedef Functor* FunctPtr;
};

inline void child(Functor& f)
{
    f.accumulate();
}
