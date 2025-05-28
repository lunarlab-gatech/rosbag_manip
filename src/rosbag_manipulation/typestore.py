import glob
from pathlib import Path
from rosbags.typesys import Stores, get_typestore, get_types_from_msg
from rosbags.typesys.store import Typestore

def guess_msgtype(path: Path) -> str:
    """
    Guess message type name from path.
    
    Args:
        path (Path): Path to the message file.

    Returns:
        str: The guessed message type name.
    """
    name = path.relative_to(path.parents[2]).with_suffix('')
    if 'msg' not in name.parts:
        name = name.parent / 'msg' / name.name
    return str(name)

def get_typestore_with_external_msgs(external_msgs_path: str) -> Typestore:
    """
    Get a Typestore with external message types added from the specified path.

    Args:
        external_msgs_path (str): Path to the folder containing external message types.

    Returns:
        Typestore: A Typestore instance with the external message types added.
    """
    typestore = get_typestore(Stores.ROS2_HUMBLE)
    
    # Load external message types
    if external_msgs_path:
        external_msgs_path = Path(external_msgs_path)
        for msg_path in glob.glob(str(external_msgs_path / Path('**') /Path('*.msg')), recursive=True):
            msg_path = Path(msg_path)
            msgdef = msg_path.read_text(encoding='utf-8')
            typestore.register(get_types_from_msg(msgdef, guess_msgtype(msg_path)))
    
    return typestore