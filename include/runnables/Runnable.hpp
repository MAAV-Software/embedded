#ifndef EVENT_HPP_
#define EVENT_HPP_

// base class for event stuff
class Runnable
{
public:
	virtual void run() = 0;
	virtual ~Runnable() {}
};

#endif /* EVENT_HPP_ */
