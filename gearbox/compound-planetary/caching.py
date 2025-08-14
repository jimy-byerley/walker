class Cached:
	def __init__(self):
		self.memo = {}
	def exec(self, code, globals, locals):
		# run code 
		
	def cache(self, func):
		@wrap(func)
		def wrap(*args, **kwargs):
			if self.function_identity(func)
			args = inspect.signature(func)(*args, **kwargs)
			
	
