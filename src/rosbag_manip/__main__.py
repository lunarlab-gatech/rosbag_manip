from .rosbag_manip import rosbag_manipulation
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
    manipulator = rosbag_manipulation.from_yaml(yaml_path)

if __name__ == "__main__":
    app()