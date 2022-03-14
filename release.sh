if [ "$1" = "patch" ] || [ "$1" = "minor" ] || [ "$1" = "major" ]
then
    python3 -m pip install --upgrade bumpversion
    python3 -m bumpversion --config-file .bumpversion.cfg $1 
    python3 -m pip install --upgrade build 
    python3 -m build 
    twine upload -r pypi dist/*
else
    echo "Please indicate bump level: patch, minor, major"
fi