# Code Style
We use the "Peaccy" code standard for the time being (a.k.a. whatever I like because I'm the only one writing it). Here is an outline:
1. Class names are in PascalCase
2. Variable names are in camelCase
   - DON'T use m_ or p_ or whatever because I find it to be distracting and useless
   - In functions, use this.\<variable name\> if the parameter is the same name
3. Constants should be in MACRO_CASE but I kinda didn't do that here (whoops).
4. Inline brackets always {}. Doing the stupid separate line thing is hideous and a waste of space.
> do it like this {
>    
>}

> never this garbage\
> {
>
> }
5. Think about how the function you're writing will be used. Try to make everything read like normal english as much as possible.
    - remember that your class names should be nouns, and functions should be verbs.
    e.g. DriveTrain.drive() instead of something like DriveTrainRunner.go() (the second one is redundant and doesn't read well)