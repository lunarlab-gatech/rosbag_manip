from decimal import Decimal
import numpy as np
from typeguard import typechecked

@typechecked
def convert_collection_into_decimal_array(collection: np.ndarray | list) -> np.ndarray[Decimal]:
    """
    This helper method maps collections (such as arrays or lists) into 
    an np.ndarray of Decimal objects for easy operations and high-fidelity
    numbers.

    Args:
        collection (np.ndarray | list): A sequential collection of numbers.
    Returns:
        np.ndarray[Decimal]: A numpy array with Decimal objects.
    """

    if type(collection) == list: return np.array(collection, dtype=Decimal)
    elif collection.dtype != Decimal: return collection.astype(Decimal)
    else: return collection