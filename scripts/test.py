correction_count 	= 0
max_correction_run 	= 15


class Job:
      name = 'test'
      average = 0.0
      values = None # list cannot be initialized here!
      def __init__(self, name):
            self.name = name   



def test():
      global correction_count,max_correction_run
      correction_count 	      = 2
      max_correction_run      = 1
      

s1 = Job('test')
print(s1.name)
test()
print(correction_count)
print(max_correction_run)



