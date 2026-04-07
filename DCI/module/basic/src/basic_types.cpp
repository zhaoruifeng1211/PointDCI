#include "basic_types.h"

basic_types::basic_types() {
}
double basic_types::maxDifference(std::queue<int> queue_)
{
	if (queue_.size() > 0)
	{
		int max = queue_.front();
		int min = queue_.front();
		while (!queue_.empty())
		{
			if (queue_.front() > max)
			{
				max = queue_.front();
			}
			if (queue_.front() < min) {
				min = queue_.front();
			}
			queue_.pop();
		}
		return max - min;
	}
	else {
		return 0.0;
	}
}

bool basic_types::isQueueStable(std::queue<int> queue_)
{
	if (queue_.size() > 0)
	{
		int first = queue_.front();
		while (!queue_.empty())
		{
			if (queue_.front() != first)
			{
				return false;
			}
			queue_.pop();
		}
		return true;
	}
	else {
		return false;
	}
}

//template <typename T>
//bool LoggerFactory<T>::logger_initialized = false;
