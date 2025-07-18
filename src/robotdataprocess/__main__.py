from .CmdLineInterface import CmdLineInterface
import typer

# Setup command line access
app = typer.Typer()

# Set yaml as a subcommand
@app.command()
def yaml(yaml_path: str):
    interface = CmdLineInterface.from_yaml(yaml_path)

if __name__ == "__main__":
    app()