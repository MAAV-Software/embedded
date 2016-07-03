#ifndef EVENT_HPP_
#define EVENT_HPP_

/**
 * @brief base class for event stuff
 * @details method run() is pure virtual, so children must implement this
 */
class Runnable
{
public:
	virtual void run() = 0;
	virtual ~Runnable() {}
};

#endif /* EVENT_HPP_ */
