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

	//�̳߳�,�����ύ��κ�������ķ����ʽ����������ִ��,���Ի�ȡִ�з���ֵ
	//��֧�����Ա����, ֧���ྲ̬��Ա������ȫ�ֺ���,operator()������
	class ThreadPool
	{
	public:
		using Task = std::function<void()>;

	public:
		ThreadPool(unsigned short size = 6) :_bStopped{ false }
		{
			_nThreadsNum = size < 1 ? 1 : size;
			for (size = 0; size < _nThreadsNum; ++size)
			{   //��ʼ���߳�����
				_pool.emplace_back(
					[this]
				{ // �����̺߳���
					while (!this->_bStopped)
					{
						std::function<void()> task;
						{   // ��ȡһ����ִ�е� task
							std::unique_lock<std::mutex> lock{ this->_lock };// unique_lock ��� lock_guard �ĺô��ǣ�������ʱ unlock() �� lock()
							this->_taskCondition.wait(lock, [this] {return this->_bStopped.load() || !this->_tasks.empty(); }); // wait ֱ���� task
							//_bStopped=false��_tasksΪ��ʱ������Ϊfalse��_taskCondition�Ż�������ֱ�������߳�notify����
							if (this->_bStopped && this->_tasks.empty())
								return;//_bStopped=true��_tasksΪ�գ����˳��̺߳���
							task = std::move(this->_tasks.front()); // ȡһ�� task
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
			_taskCondition.notify_all(); // ���������߳�ִ��
			for (std::thread& thread : _pool) {
				//thread.detach(); // ���̡߳���������
				if (thread.joinable())
					thread.join(); // �ȴ���������� ǰ�᣺�߳�һ����ִ����
			}
		}
		//�����߳�����
		const int& threadsCount() { return _nThreadsNum; }

		//ֹͣ�̳߳�
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

			using RetType = decltype(f(args...)); // typename std::result_of<F(Args...)>::type, ���� f �ķ���ֵ����
			auto task = std::make_shared<std::packaged_task<RetType()>>
				(std::bind(std::forward<F>(f), std::forward<Args>(args)...)); 

			/*std::packaged_task<RetType()>* task = new std::packaged_task<RetType()>(std::bind(std::forward<F>(f),
			std::forward<Args>(args)...));*/
			std::future<RetType> future = task->get_future();
			{    // ������񵽶���
				std::lock_guard<std::mutex> lock{ _lock };//�Ե�ǰ���������  lock_guard �� mutex �� stack ��װ�࣬�����ʱ�� lock()��������ʱ�� unlock()
				_tasks.emplace([task]()
				{ // push(Task{...})
					(*task)();
				});
			}
			_taskCondition.notify_one(); // ����һ���߳�ִ��

			return future;
		}

	

	public:
		// �̳߳�
		std::vector<std::thread>		_pool;
		// �������
		std::queue<Task>				_tasks;
		// ͬ��
		std::mutex						_lock;
		// ��������
		std::condition_variable			_taskCondition;
		// �Ƿ�ر��ύ
		std::atomic<bool>			    _bStopped;
		//�����߳�����
		std::atomic<int>				_nThreadsNum;

	};

}

#endif