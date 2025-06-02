from .rosbag_manipulator import rosbag_Manipulator
import typer

# Setup command line access
app = typer.Typer()

# Set yaml as a subcommand
@app.command()
def yaml(yaml_path: str):
    """
    Create a rosbag manipulator from a YAML file, and then run the manipulation process.

    Args:
        yaml_path (str) : Path to the YAML file containing the manipulation configuration.
    """
    manipulator = rosbag_Manipulator.from_yaml(yaml_path)

if __name__ == "__main__":
    app()