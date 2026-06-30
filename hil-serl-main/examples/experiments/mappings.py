from importlib import import_module


class LazyTrainConfig:
    def __init__(self, module_name, class_name="TrainConfig"):
        self.module_name = module_name
        self.class_name = class_name

    def __call__(self, *args, **kwargs):
        return self._load()(*args, **kwargs)

    def __repr__(self):
        return f"<LazyTrainConfig {self.module_name}.{self.class_name}>"

    def _load(self):
        module = import_module(self.module_name)
        return getattr(module, self.class_name)

CONFIG_MAPPING = {
                "ram_insertion": LazyTrainConfig("experiments.ram_insertion.config"),
                "usb_pickup_insertion": LazyTrainConfig("experiments.usb_pickup_insertion.config"),
                "object_handover": LazyTrainConfig("experiments.object_handover.config"),
                "egg_flip": LazyTrainConfig("experiments.egg_flip.config"),
                "hemisphere_insertion": LazyTrainConfig("experiments.hemisphere_insertion.config"),
               }
