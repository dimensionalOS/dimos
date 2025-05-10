# Copyright (c) Facebook, Inc. and its affiliates.
from detectron2.config import CfgNode as CN

def add_detic_config(cfg):
    _C = cfg

    _C.WITH_IMAGE_LABELS = False # Turn on co-training with classification data

    # Open-vocabulary classifier
    _C.MODEL.ROI_BOX_HEAD.USE_ZEROSHOT_CLS = False # Use fixed classifier for open-vocabulary detection
    _C.MODEL.ROI_BOX_HEAD.ZEROSHOT_WEIGHT_PATH = 'datasets/metadata/lvis_v1_clip_a+cname.npy'
    _C.MODEL.ROI_BOX_HEAD.ZEROSHOT_WEIGHT_DIM = 512
    _C.MODEL.ROI_BOX_HEAD.NORM_WEIGHT = True
    _C.MODEL.ROI_BOX_HEAD.NORM_TEMP = 50.0
    _C.MODEL.ROI_BOX_HEAD.IGNORE_ZERO_CATS = False
    _C.MODEL.ROI_BOX_HEAD.USE_BIAS = 0.0 # >= 0: not use
    
    _C.MODEL.ROI_BOX_HEAD.MULT_PROPOSAL_SCORE = False # CenterNet2
    _C.MODEL.ROI_BOX_HEAD.USE_SIGMOID_CE = False
    _C.MODEL.ROI_BOX_HEAD.PRIOR_PROB = 0.01
    _C.MODEL.ROI_BOX_HEAD.USE_FED_LOSS = False # Federated Loss
    _C.MODEL.ROI_BOX_HEAD.CAT_FREQ_PATH = \
        'datasets/metadata/lvis_v1_train_cat_info.json'
    _C.MODEL.ROI_BOX_HEAD.FED_LOSS_NUM_CAT = 50
    _C.MODEL.ROI_BOX_HEAD.FED_LOSS_FREQ_WEIGHT = 0.5

    # Classification data configs
    _C.MODEL.ROI_BOX_HEAD.IMAGE_LABEL_LOSS = 'max_size' # max, softmax, sum
    _C.MODEL.ROI_BOX_HEAD.IMAGE_LOSS_WEIGHT = 0.1
    _C.MODEL.ROI_BOX_HEAD.IMAGE_BOX_SIZE = 1.0
    _C.MODEL.ROI_BOX_HEAD.ADD_IMAGE_BOX = False # Used for image-box loss and caption loss
    _C.MODEL.ROI_BOX_HEAD.WS_NUM_PROPS = 128 # num proposals for image-labeled data
    _C.MODEL.ROI_BOX_HEAD.WITH_SOFTMAX_PROP = False # Used for WSDDN
    _C.MODEL.ROI_BOX_HEAD.CAPTION_WEIGHT = 1.0 # Caption loss weight
    _C.MODEL.ROI_BOX_HEAD.NEG_CAP_WEIGHT = 0.125 # Caption loss hyper-parameter
    _C.MODEL.ROI_BOX_HEAD.ADD_FEATURE_TO_PROP = False # Used for WSDDN
    _C.MODEL.ROI_BOX_HEAD.SOFTMAX_WEAK_LOSS = False # Used when USE_SIGMOID_CE is False

    _C.MODEL.ROI_HEADS.MASK_WEIGHT = 1.0
    _C.MODEL.ROI_HEADS.ONE_CLASS_PER_PROPOSAL = False # For demo only

    # Caption losses
    _C.MODEL.CAP_BATCH_RATIO = 4 # Ratio between detection data and caption data
    _C.MODEL.WITH_CAPTION = False
    _C.MODEL.SYNC_CAPTION_BATCH = False # synchronize across GPUs to enlarge # "classes"

    # dynamic class sampling when training with 21K classes
    _C.MODEL.DYNAMIC_CLASSIFIER = False
    _C.MODEL.NUM_SAMPLE_CATS = 50

    # Different classifiers in testing, used in cross-dataset evaluation
    _C.MODEL.RESET_CLS_TESTS = False
    _C.MODEL.TEST_CLASSIFIERS = []
    _C.MODEL.TEST_NUM_CLASSES = []

    # Backbones
    _C.MODEL.SWIN = CN()
    _C.MODEL.SWIN.SIZE = 'T' # 'T', 'S', 'B'
    _C.MODEL.SWIN.USE_CHECKPOINT = False
    _C.MODEL.SWIN.OUT_FEATURES = (1, 2, 3) # FPN stride 8 - 32

    _C.MODEL.TIMM = CN()
    _C.MODEL.TIMM.BASE_NAME = 'resnet50'
    _C.MODEL.TIMM.OUT_LEVELS = (3, 4, 5)
    _C.MODEL.TIMM.NORM = 'FrozenBN'
    _C.MODEL.TIMM.FREEZE_AT = 0
    _C.MODEL.TIMM.PRETRAINED = False
    _C.MODEL.DATASET_LOSS_WEIGHT = []
    
    # Multi-dataset dataloader
    _C.DATALOADER.DATASET_RATIO = [1, 1] # sample ratio
    _C.DATALOADER.USE_RFS = [False, False]
    _C.DATALOADER.MULTI_DATASET_GROUPING = False # Always true when multi-dataset is enabled
    _C.DATALOADER.DATASET_ANN = ['box', 'box'] # Annotation type of each dataset
    _C.DATALOADER.USE_DIFF_BS_SIZE = False # Use different batchsize for each dataset
    _C.DATALOADER.DATASET_BS = [8, 32] # Used when USE_DIFF_BS_SIZE is on
    _C.DATALOADER.DATASET_INPUT_SIZE = [896, 384] # Used when USE_DIFF_BS_SIZE is on
    _C.DATALOADER.DATASET_INPUT_SCALE = [(0.1, 2.0), (0.5, 1.5)] # Used when USE_DIFF_BS_SIZE is on 
    _C.DATALOADER.DATASET_MIN_SIZES = [(640, 800), (320, 400)] # Used when USE_DIFF_BS_SIZE is on
    _C.DATALOADER.DATASET_MAX_SIZES = [1333, 667] # Used when USE_DIFF_BS_SIZE is on
    _C.DATALOADER.USE_TAR_DATASET = False # for ImageNet-21K, directly reading from unziped files
    _C.DATALOADER.TARFILE_PATH = 'datasets/imagenet/metadata-22k/tar_files.npy'
    _C.DATALOADER.TAR_INDEX_DIR = 'datasets/imagenet/metadata-22k/tarindex_npy'
    
    _C.SOLVER.USE_CUSTOM_SOLVER = False
    _C.SOLVER.OPTIMIZER = 'SGD'
    _C.SOLVER.BACKBONE_MULTIPLIER = 1.0 # Used in DETR
    _C.SOLVER.CUSTOM_MULTIPLIER = 1.0 # Used in DETR
    _C.SOLVER.CUSTOM_MULTIPLIER_NAME = [] # Used in DETR

    # Deformable DETR
    _C.MODEL.DETR = CN()
    _C.MODEL.DETR.NUM_CLASSES = 80
    _C.MODEL.DETR.FROZEN_WEIGHTS = '' # For Segmentation
    _C.MODEL.DETR.GIOU_WEIGHT = 2.0
    _C.MODEL.DETR.L1_WEIGHT = 5.0
    _C.MODEL.DETR.DEEP_SUPERVISION = True
    _C.MODEL.DETR.NO_OBJECT_WEIGHT = 0.1
    _C.MODEL.DETR.CLS_WEIGHT = 2.0
    _C.MODEL.DETR.NUM_FEATURE_LEVELS = 4
    _C.MODEL.DETR.TWO_STAGE = False
    _C.MODEL.DETR.WITH_BOX_REFINE = False
    _C.MODEL.DETR.FOCAL_ALPHA = 0.25
    _C.MODEL.DETR.NHEADS = 8
    _C.MODEL.DETR.DROPOUT = 0.1
    _C.MODEL.DETR.DIM_FEEDFORWARD = 2048
    _C.MODEL.DETR.ENC_LAYERS = 6
    _C.MODEL.DETR.DEC_LAYERS = 6
    _C.MODEL.DETR.PRE_NORM = False
    _C.MODEL.DETR.HIDDEN_DIM = 256
    _C.MODEL.DETR.NUM_OBJECT_QUERIES = 100

    _C.MODEL.DETR.USE_FED_LOSS = False
    _C.MODEL.DETR.WEAK_WEIGHT = 0.1

    _C.INPUT.CUSTOM_AUG = ''
    _C.INPUT.TRAIN_SIZE = 640
    _C.INPUT.TEST_SIZE = 640
    _C.INPUT.SCALE_RANGE = (0.1, 2.)
    # 'default' for fixed short/ long edge, 'square' for max size=INPUT.SIZE
    _C.INPUT.TEST_INPUT_TYPE = 'default' 

    _C.FIND_UNUSED_PARAM = True
    _C.EVAL_PRED_AR = False
    _C.EVAL_PROPOSAL_AR = False
    _C.EVAL_CAT_SPEC_AR = False
    _C.IS_DEBUG = False
    _C.QUICK_DEBUG = False
    _C.FP16 = False
    _C.EVAL_AP_FIX = False
    _C.GEN_PSEDO_LABELS = False
    _C.SAVE_DEBUG_PATH = 'output/save_debug/'