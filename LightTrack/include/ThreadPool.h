#ifndef THREAD_POOL_H
#define THREAD_POOL_H

#include <vector>
#include <queue>
#include <thread>
#include <atomic>
#include <condition_variable>
#include <future>
#include <functional>
#include <stdexcept>

namespace SLAM
{
#define  MAX_THREAD_NUM 256

	//线程池,可以提交变参函数或拉姆达表达式的匿名函数执行,可以获取执行返回值
	//不支持类成员函数, 支持类静态成员函数或全局函数,operator()函数等
	class ThreadPool
	{
	public:
		using Task = std::function<void()>;

	public:
		ThreadPool(unsigned short size = 6) :_bStopped{ false }
		{
			_nThreadsNum = size < 1 ? 1 : size;
			for (size = 0; size < _nThreadsNum; ++size)
			{   //初始化线程数量
				_pool.emplace_back(
					[this]
				{ // 工作线程函数
					while (!this->_bStopped)
					{
						std::function<void()> task;
						{   // 获取一个待执行的 task
							std::unique_lock<std::mutex> lock{ this->_lock };// unique_lock 相比 lock_guard 的好处是：可以随时 unlock() 和 lock()
							this->_taskCondition.wait(lock, [this] {return this->_bStopped.load() || !this->_tasks.empty(); }); // wait 直到有 task
							//_bStopped=false且_tasks为空时条件才为false，_taskCondition才会阻塞，直到其它线程notify唤醒
							if (this->_bStopped && this->_tasks.empty())
								return;//_bStopped=true且_tasks为空，才退出线程函数
							task = std::move(this->_tasks.front()); // 取一个 task
							this->_tasks.pop();
						}
						_nThreadsNum--;
						task();
						_nThreadsNum++;
					}
				}
				);
			}
		}
		~ThreadPool() { stop(); }

		void stop()
		{
			std::unique_lock<std::mutex> lock{ this->_lock };
			while (!_tasks.empty())
				_tasks.pop();
			lock.unlock();

			_bStopped.store(true);
			_taskCondition.notify_all(); // 唤醒所有线程执行
			for (std::thread& thread : _pool) {
				//thread.detach(); // 让线程“自生自灭”
				if (thread.joinable())
					thread.join(); // 等待任务结束， 前提：线程一定会执行完
			}
		}
		//空闲线程数量
		const int& threadsCount() { return _nThreadsNum; }

		//停止线程池
		void forceClear()
		{
			std::unique_lock<std::mutex> lock{ this->_lock };
			while (!_tasks.empty())
				_tasks.pop();
			lock.unlock();
			_taskCondition.notify_all();
			//std::queue<Task> empty;
			//std::swap(empty, _tasks);		
		}

	

		template<class F, class... Args>
		auto commit(F&& f, Args&&... args) ->std::future<decltype(f(args...))>
		{
			if (_bStopped.load())    // stop == true ??
				throw std::runtime_error("commit on ThreadPool is stopped.");

			using RetType = decltype(f(args...)); // typename std::result_of<F(Args...)>::type, 函数 f 的返回值类型
			auto task = std::make_shared<std::packaged_task<RetType()>>
				(std::bind(std::forward<F>(f), std::forward<Args>(args)...)); 

			/*std::packaged_task<RetType()>* task = new std::packaged_task<RetType()>(std::bind(std::forward<F>(f),
			std::forward<Args>(args)...));*/
			std::future<RetType> future = task->get_future();
			{    // 添加任务到队列
				std::lock_guard<std::mutex> lock{ _lock };//对当前块的语句加锁  lock_guard 是 mutex 的 stack 封装类，构造的时候 lock()，析构的时候 unlock()
				_tasks.emplace([task]()
				{ // push(Task{...})
					(*task)();
				});
			}
			_taskCondition.notify_one(); // 唤醒一个线程执行

			return future;
		}

	

	public:
		// 线程池
		std::vector<std::thread>		_pool;
		// 任务队列
		std::queue<Task>				_tasks;
		// 同步
		std::mutex						_lock;
		// 条件阻塞
		std::condition_variable			_taskCondition;
		// 是否关闭提交
		std::atomic<bool>			    _bStopped;
		//空闲线程数量
		std::atomic<int>				_nThreadsNum;

	};

}

#endif