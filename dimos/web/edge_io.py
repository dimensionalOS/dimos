from reactivex.disposable import CompositeDisposable

class EdgeIO():
    def __init__(self, dev_name:str="NA", edge_type:str="Base"):
        self.dev_name = dev_name
        self.edge_type = edge_type
        self.disposables = CompositeDisposable()

    def dispose_all(self):
        """Disposes of all active subscriptions managed by this agent."""
        self.disposables.dispose()
