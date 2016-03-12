#ifndef MAAV_CTRL_HPP
#define MAAV_CTRL_HPP

/**
 * @file
 * @brief This file controls the text on Main Page and Related Pages
 *
 * @details Edit this file to edit the doxygen documentatoin
 */

/**
 * @mainpage CTRL
 *
 * @tableofcontents
 *
 * @section intro Introduction
 *
 * There are two important binaries are created as a part of the build:
 *
 * * nav: The navigation server. Runs on the vehicle and does competition
 * logic. More about it can be read here: @ref nav.
 * * gcs: The ground control station. Contains a lot of functionality. More
 * about it can be read here: @ref gcs.
 *
 * To get a grip of the architecture, take a look at @ref arch. For info about
 * camera calibration, check out @ref calibration.
 *
 * @section contrib Contributing
 *
 * Check out page @ref contributing. After that, read @ref stylecpp. Finally,
 * become addicted to coffee.
 */

/**
 * @page arch NAV architecture
 *
 * @tableofcontents
 *
 * @section arena Arena
 *
 * The Arena is the global state of the program. As such, all of its functions
 * are (or should be) thread safe. The forward prediction function of the
 * arena does not modify any state, which allows matching sensor data (which
 * may be time-intensive) while still relying on the best-known positions of
 * all objects in the arena.
 *
 * In addition, all of the functions of Arena (should) have locking minimized
 * as much as possible. Any thread that writes to any state will write-lock
 * that state - Arena will not use any read-lock-to-write-lock upgrades, as
 * they can work but are shaky at best.
 *
 * Lastly, the reference frame; MAAV's reference frame is the traditional
 * aerospace reference frame. Looking down at the arena from above:
 *
 * * The origin is the bottom left corner
 * * The x-axis points up
 * * The y-axis points right
 * * The z-axis points into the page
 * * Roll is around the x-axis
 * * Pitch is around the y-axis
 * * Yaw is around the z-axis
 * * Rotations follow the right-hand rule
 *
 * @section eventdriven Event-driven Data Acquisition
 *
 * The sensors are each read in their own thread, and take callbacks that
 * process data as it arrives. This is important for being able to interact
 * with Gazebo - the processing callbacks are the exact same, but the threads
 * are reading from different sensors.
 *
 * @section pic Architecture Diagram
 *
 * The below figure shows the logical flow of data and instructions through
 * the nav code. Color is used to group tasks that run in the same thread.
 *
 * @htmlonly
 * <center><img src="../hlsys.png" /></center>
 * @endhtmlonly
 */

/**
 * @page nav The 'nav' binary
 *
 * @tableofcontents
 *
 * @section summary Summary
 *
 * The 'nav' binary is the main binary that is run on the vehicle. It
 * additionally understands a number of useful options for debugging on a
 * laptop. Consult the '--help' option for a full list of options that can be
 * passed.
 *
 * @section test_serial_comms Testing Serial Communications
 *
 * Running
 *
 *     src/nav --no-laser --debug
 *
 * will launch the server and only connect to the controller. Unless you
 * modify the code, you will also have to launch the ground control station
 * by running:
 *
 *     src/gcs
 *
 * Once the ground control is up, open the command UI, connect to
 * 127.0.0.1:9000 and send poses. That should generate some stuff for testing
 * the serial communications.
 */

/**
 * @page contributing Contributing
 *
 * This document isn't an intro to git. If you want/need that, read [Pro
 * Git](http://git-scm.com/book) by Scott Chacon. It'll teach more about git
 * than anyone on MAAV can tell you, and probably do a better job. Mr. Chacon
 * has been using git for years and works at Github. The man knows.
 *
 * The typical workflow looks like this (omit the redmine steps if you do not
 * have access yet):
 *
 * 1. On Redmine, ensure the corresponding issue is assigned to yourself, has
 *    a due date, and is "in progress".
 * 2. Checkout a new, temporary branch to work on a certain issue. Do **not**
 *    work on the master branch. Your work most likely will not be accepted.
 * 3. Work on that issue, committing every so often (probably at least
 *    once/hour). If it takes a few days, add notes on the corresponding issue
 *    on Redmine so that everyone can follow what's going on.
 * 4. Push all work to your personal repo.
 * 5. Mark the issue as resolved in Redmine, with a note explaining the changes
 *    and what branch its on.
 *
 * When a "manager" sees email that your issue has been marked resolved, it
 * will be reviewed, critiqued, and merged.
 *
 * Redmine access is granted after you prove yourself.
 */

/**
 * @page stylecpp MAAV C++ Coding Style Guide
 *
 * @tableofcontents
 *
 * @section intro Introduction
 *
 * Why do we have this style guide here? Because coding can be very unenjoyable
 * sometimes, and working with badly written code is even more unenjoyable. In
 * addition, merging in work when different authors use different style is much
 * more difficult than merging in work from authors who use the same style.
 * Getting a complex piece of software working is inherently difficult, so we
 * want our team members to be in their best emotional states when they are
 * coding. Since seeing badly written software makes us angry, we would like
 * all (coding) team members to follow this style guide when they code, so as
 * to produce software that is easy to read, work with, maintain, and extend.
 * This style guide is based on the following concepts:
 *
 * <ol>
 * <li><b>The Concept of DRY</b><br />
 * Every piece of knowledge must have a <b>single</b>, unambiguous,
 * authoritative representation within a piece of software. This is known as the
 * Do not Repeat Yourself (DRY) principle. Minimize the amount of duplicate code
 * in your software, or we will get mad at you!<br />
 * For a thorough discussion about the principle of DRY, refer to section 7 in
 * <i>The Pragmatic Programmer</i> by Andrew Hunt and David Thomas.</li>
 *
 * <li><b>The Concept of Code Reliability</b><br />
 * Many bugs in software can be prevented by coding in a simple, clear, and
 * consistent style that follows idioms and patterns distilled by programming
 * "gurus." This style guide summarizes many advices from various C++ gurus
 * such as Scott Meyers and Bjarne Stroustrup, so conforming to this style
 * guide will most likely improve your code reliability.</li>
 *
 * <li><b>The Concept of Not Looking Clueless</b><br />
 * You know, your fellow teammates are all intelligent human beings. We can
 * find out who wrote which piece of code. If you don't want to look like an
 * idiot in front of the whole team, you should probably follow this style
 * guide.</li>
 * </ol>
 *
 * We think that we just gave you some good reasons for following this style
 * guide. What, you are not convinced? Well, if you do not follow this style
 * guide, we will not merge in your code. End of story. <b>So you better follow
 * this style guide!</b>
 *
 * @section general General Rules
 *
 * This section covers general rules such as the formatting and commenting.
 *
 * @subsection whitespace Whitespace
 *
 * <ol>
 * <li>Spaces around operators such as <code>+</code>, <code>-</code>,
 * <code>></code>, <code><</code>, <code>*</code>, <code>/</code>,
 * <code>>=</code>, <code><=</code>, <code>=</code>, <code>==</code>:
 * @code
 * 		int x = 4 + 5;
 * @endcode
 * </li>
 * <li>Spaces around keywords such as <code>if</code>, <code>else</code>,
 * <code>else if</code>, <code>while</code>, <code>for</code>, <code>do</code>,
 * <code>switch</code>:
 * @code
 * 		if (whatever()) //Notice that space
 * 			;
 * @endcode
 * </li>
 * <li>No spaces around functions or operators that feel like functions such as
 * <code>sizeof()</code>, <code>typeof()</code>, <code>alignof()</code>, and
 * C++ style casts:
 * @code
 * 		int size = static_cast<int>(v.size());	// where v is a std::vector
 * @endcode
 * </li>
 * <li>
 * Inside parentheses, there should be no spaces by the parentheses, but there
 * should be spacing between arguments. For example:
 * @code
 * 		tmp = a_function(blah, arg2);		//Good
 * 		tmp = a_function( blah, arg2 );		//Bad
 * 		tmp = a_function(blah,arg2);		//Bad
 * @endcode
 * </li>
 * <li>Space after C style casts:
 * @code
 * 		int size = (int) v.size();	// where v is a std::vector
 * @endcode
 * </li>
 * <li>Blank lines should contain no whitespace.</li>
 * <li>Lines should not have any trailing whitespace.</li>
 * </ol>
 *
 * @subsection indentation Indentation
 *
 * Use tabs that are 4 spaces in the width.
 *
 * You <i>must</i> follow this rule. Anyone that has gone through EECS 280 has
 * seen the 2-character space indents used by Professors DeOrio and Fu. While
 * this looks good on slides, it's <i>not fun at all</i> in real life.<br />
 *
 * Proper indentation for declaring C++ classes:
 * @code
 *		class Blah
 *		{
 *		public:
 *			void whatever();
 *
 *		private:
 *			void privateStuff();
 *		}
 * @endcode
 *
 * Proper indentation for switch statements
 * @code
 *		switch(tmp)
 *		{
 *		case 10:
 *			// stuff
 *			break;
 *		case 9:
 *			// stuff
 *			// Fall Through
 *		case 2:
 *			//stuff
 *			break;
 *		default:
 *	 		//stuff
 *		 	break;
 *		}
 * @endcode
 *
 * @subsection breaking_lines Breaking Lines
 *
 * Limit lines to 80 characters. This is a hard limit unless going past this
 * significantly enhances readability or breaks a user-visible string like a log
 * message.
 *
 * Breaking a line can be formatted either of these ways:
 * @code
 *		int tmp = super_long_function_call(long_variable_one, long_variable_two,
 *			long_variable_three, long_variable_four);
 * @endcode
 * or
 * @code
 *		int super_long_function_call(
 *			arg_one,
 *			arg_two,
 *			arg_three
 *		);
 * @endcode
 *
 * @subsection braces Braces
 *
 * There are many legitimate styles for placing braces. We allow the following
 * two styles:
 * <ol>
 * <li><b>Kernighan and Ritchie (K&R) style</b><br />
 * This is the style used in the book <i>The C Programming Language</i> by Brian
 * W. Kernighan and Dennis M. Ritchie.</li>
 * <li><b>Allman style</b>, also know as <b>BSD style</b><br />
 * This style is named after Eric Allman, who implemented many of the utilities
 * for BSD Unix.</li>
 * </ol>
 *
 * For functions:
 * @code
 *		// Both K&R and Allman
 *		int func_definition(int blah)
 *		{
 *			//stuff
 *		}
 * @endcode
 *
 * For <code>if</code> statements, <code>for</code> statements, and <code>
 * while</code> statements, etc.:
 * @code
 *		// K&R
 *		if (blah == 2) {
 *			//Do stuff
 *			//Lots of stuff
 *		} else if (blah == 1) {
 *			//Do different stuff
 *			//Lots of stuff
 *		} else {
 *			//stuff
 *			//Lots of stuff
 *		}
 *
 *		while (blah) {
 *			//stuff
 *			//Lots of stuff
 *		}
 *
 *		// Allman
 *		if (blah == 2)
 *		{
 *			//Do stuff
 *			//Lots of stuff
 *		}
 *		else if (blah == 1)
 *		{
 *			//Do different stuff
 *			//Lots of stuff
 *		}
 *		else
 *		{
 *			//Stuff
 *			//Lots of stuff
 *		}
 *
 *		while (blah)
 *		{
 *			//Shtuff
 *			//Lots of stuff
 *		}
 * @endcode
 *
 * Traditionally, for both styles, if you only have one-liners it's okay to omit
 * the braces. However, do not do it. If you see legacy code that omit braces
 * for one-liners, add the braces.
 *
 * All other styles, such as Stroustrup style, GNU style, Linux kernel style,
 * Whitesmiths style, Lisp style, Horstmann style, Pico style, Ratliff style,
 * are all not allowed. Also, always follow the style that already exists in
 * the file you are editing.
 *
 * @subsection commenting Commenting
 *
 * The purpose of comments is to help people understand your code: not just
 * other people, but also yourself. If you poorly comment your code, chances are
 * you will have a harder time writing your code. Also, if you ask us for help
 * with your code, and your code is poorly commented and hardly readable, we
 * reserve the rights to get angry at you and send you away to a corner, where
 * you will clean your code up.
 *
 * However, do not overcomment your code. More specifically, do not write
 * comments that are completely synonomous with the code like:
 * @code
 *		// add a to i and store value in i
 *		i += a;
 *		// call foo with i
 *		foo(i);
 * @endcode
 *
 * If you are confused as for what you should and should not comment, Professor
 * Kieras has wrote a very good handout on commenting. You can access it
 * <a href="http://umich.edu/~eecs381/handouts/comments.pdf" target="_blank">
 * here</a>. However, do not follow his indentation styles.
 *
 * We're using Doxygen to generate pretty HTML documentation based on JavaDoc
 * syntax. Here are some rules about this:
 * <ol>
 * <li>For classes, please fill out at least the <code>brief</code>,
 * <code>details</code>, and <code>author</code> tags. There can be multiple
 * <code>author</code> tags. Please keep them at the class level, and add
 * yourself as an author only if you make a significant contribution.</li>
 * <li>For methods, please use at least the <code>brief</code>,
 * <code>param</code>, and <code>return</code> tags.</li>
 * <li>If there are pre and post conditions to your method, then also you can
 * add <code>pre</code> and <code>post</code> tags in the comment.</li>
 * </ol>
 *
 * @subsection commit_msg Commit Messages
 * Commit messages should be formmatted as follows (thanks to Tim Pope from
 * <a href="http://tpo.pe/" target="_blank">tpo.pe</a> for this template):
 * @code{.unparsed}
 * Bug #<bug number here> - Summary of bug
 *
 * Detailed explanatory text, if necessary.  Wrap it to about 72 characters or
 * so.  In some contexts, the first line is treated as the subject of an email
 * and the rest of the text as the body.  The blank line separating the
 * summary from the body is critical (unless you omit the body entirely);
 * tools like rebase can get confused if you run the two together.
 *
 * Further paragraphs come after blank lines.
 *
 * - Bullet points are okay, too
 * * Typically a hyphen or asterisk is used for the bullet, preceded by a
 *   single space, with blank lines in between, but conventions vary here
 * @endcode
 *
 * Scott Chacon recommends using the imperative present tense in commit messages.
 * In other words, write things as commands. Instead of writing "I added ..." or
 * "Adding ...", use "Add ...".
 *
 * Follow this within reason. If you change something so simple it only requires
 * a one sentence summary, just write a sentence.
 *
 * @section file_organization File Organization
 *
 * (Note: This section is adapted from Professor Kieras' handout on
 * <a href=http://umich.edu/~eecs381/handouts/CppHeaderFileGuidelines.pdf
 * target = "_blank">C++ header files</a>.)
 *
 * C++ programs normally take the form of a collection of separately compiled
 * modules. The contents of a module consist of user defined types, global
 * variables, and functions. The functions themselves are normally defined in a
 * source file (a “.cpp” file). Each source file has a header file
 * (a “.h” file traditionally, but use we use ".hpp" files) associated with it
 * that provides the declarations needed by other modules to make use of this
 * module. The idea is that other modules can access the functionality in module
 * <code>X</code> simply by <code>#include</code>ing the <code>X.hpp</code>
 * header file, and the linker will do the rest. The code in <code>X.cpp</code>
 * needs to be compiled only the first time or if it is changed; the rest of the
 * time, the linker will link <code>X</code>’s code into the final executable
 * without needing to recompile it, which enables the Unix <code>make</code>
 * utility and IDEs to work very efficiently.  Usually, the <code>main</code>
 * module does not have a corresponding header file, since it normally uses
 * functionality in the other modules, rather than providing functionality to
 * them.
 *
 * A well organized C++ program has a good choice of modules, and properly
 * constructed header files that make it easy to understand and access the
 * functionality in a module. Well-designed header files reduce the need to
 * recompile the source files for components whenever changes to other
 * components are made. The trick is reduce the amount of “coupling” between
 * components by minimizing the number of header files that a module’s header
 * file itself <code>#include</code>s. On very large projects (like MAAV's
 * navigation code), minimizing coupling can make a huge difference in build
 * time as well as simplifying the code organization and debugging.
 *
 * The following guidelines summarize how to set up your header and source files
 * for the greatest clarity and compilation convenience.
 * <ol>
 * <li>Each module should correspond to a clear piece of functionality.
 * Conceptually, a module is a group of declarations and functions can be
 * developed and maintained separately from other modules, and perhaps even
 * reused in entirely different projects. Don’t force together into a module
 * things that will be used or maintained separately, and don’t separate
 * things that will always be used and maintained together. The Standard Library
 * modules <cmath> and <string> are good examples of clearly distinct modules.
 * </li>
 * <li>Always use include guards in a header file.
 * The most compact form of include guards uses <code>ifndef</code>. Choose a
 * guard symbol based on the header file name. Follow the convention of making
 * the symbol all-caps. For example <code>Roomba.hpp</code> would start with:
 * @code
 *		#ifndef MAAV_ROOMBA_HPP		// Note the word MAAV
 *		#define MAAV_ROOMBA_HPP
 * @endcode
 * and end with:
 * @code
 * 		#endif // MAAV_ROOMBA_HPP
 * @endcode
 * </li>
 * <li>All of the declarations needed to use a module must appear in its header
 * file, and this file is always used to access the module.</b> Thus
 * <code>#include</code>ing the header file provides all the information
 * necessary for code using the module to compile and link correctly.
 * Furthermore, if module <code>A</code> needs to use module <code>X</code>’s
 * functionality, it should always <code>#include “X.h”</code>, and never
 * contain hard-coded declarations for structs, classes, globals, or functions
 * that appear in module <code>X</code>. Why? If module X is changed, but you
 * forget to change the hard-coded declarations in module <code>A</code>, module
 * <code>A</code> could easily fail with subtle run-time errors that won’t be
 * detected by either the compiler or linker!!! Always referring to a module
 * through its header file ensures that only a single set of declarations needs
 * to be maintained.</li>
 * <li>Do not put any form of <code>using</code> statement at the top level in a
 * header file. The reason? Anybody wanting to use your module has to
 * <code>#include</code> your header file. If you have <code>using</code>
 * statements in it, then these statements become part of their code, appearing
 * at the point your header file was <code>#include</code>d. They are stuck with
 * whatever namespace decision you made, and can’t override it with their own.
 * Furthermore, the <code>using</code> statement will take effect at the point
 * where it appears in the code that <code>#include</code>d the header, meaning
 * that any code appearing before that might get treated differently from code
 * appearing after that point. There might be a hodgepodge of which headers and
 * code gets interpreted in terms of your namespace decision. A single
 * <code>using namespace std;</code> statement in a single header file in a
 * complex project can make a mess out of the namespace management for the whole
 * project. So, no top level <code>using</code> statements in a header file!
 * <br />
 * Narrowly-scoped <code>using</code> statements are OK but discouraged.
 * Occasionally it is useful or necessary to have a <code>using</code> statement
 * whose scope is within a class declaration or function definition. Because
 * these statements have a limited scope, they do not affect the entire
 * compilation unit, and so present no problem.
 * <br />
 * Just to be clear, by <code>using</code> statements, we mean namespace
 * <code>using</code> statements, not type alias statements.
 * </li>
 * <li>
 * The header file should contain only declarations, templates, and inline
 * function definitions, and is included by the source file for the module.
 * Put structure and class declarations, function prototypes, and global
 * variable <code>extern</code> declarations, in the header file; put the
 * function definitions and global variable definitions and initializations in
 * the source file. The source file for a module must include the header file;
 * the compiler will detect any discrepancies between the two, and thus help
 * ensure consistency. <br />
 * Note that for templates, unless you are using explicit instantiations
 * (which is quite rare), the compiler must have the full definitions available
 * in order to instantiate the template, and so all templated function
 * definitions must appear in the header file. Similarly, the compiler must have
 * the full definitions available for ordinary (nonmember) functions that need
 * to be <code>inline</code>d, so the definitions for <code>inline</code>
 * functions will also appear (declared <code>inline</code>) in the header file.
 * </li>
 * <li>Set up global variables for a module with an <code>extern</code>
 * declaration in the header file, and a defining declaration in the source
 * file. For global variables that will be known throughout the program, place
 * an <code>extern</code> declaration in the header file, as in:
 * @code
 *		extern int NUMBER_OF_ENTITIES;
 * @endcode
 * The other modules <code>#include</code> only the header file. The source file
 * for the module must include this same header file, and near the beginning of
 * the file, a defining declaration should appear - this declaration both
 * defines and initializes the global variables, as in:
 * @code
 *		int NUMBER_OF_ENTITIES = 0;
 * @endcode
 * Of course, some other value besides zero could be used as the initial value,
 * and <code>static</code>/global variables are initialized to zero by default;
 * but initializing explicitly to zero is customary.</li>
 * <li> Keep a module’s internal declarations out of the header file. The header
 * file is supposed to contain the public interface for the module, and
 * everything else is supposed to be hidden from outside view and access. Thus,
 * if you need class or struct declarations, global variables, templates, or
 * functions that are used only in the code in the source file, put their
 * definitions or declarations at convenient points in the <code>.cpp</code>
 * file and do not mention them in the <code>.hpp</code> file. One common
 * example is special-purpose function object class declarations for use with
 * the Standard Library algorithms. Often these have absolutely no value outside
 * the module, and so should not be simply tossed into the header file with
 * other class declarations. It is better to place their declarations in the
 * <code>.cpp</code> file just ahead of the function that uses them.
 * Furthermore, internal declarations should be <code>static</code> so that they
 * will be given internal linkage. (Note that constants declared as
 * <code>const</code> variables with initialization automatically get internal
 * linkage, even if they appear in a header file, so declaring these as
 * <code>static</code> is redundant and should not be done. This way, other
 * modules do not (and can not) know about these internal declarations.</li>
 * <li> Put declarations in the narrowest scope possible in the header file.
 * Double-check Guideline #5 and make sure that a declaration belongs in the
 * header file rather than the source file for a module. If it does belong in
 * the header file, place the declaration in the private section of a class if
 * possible, followed by the protected section, followed by the public section
 * of a class. Do not make it top-level in the header file unless it really
 * needs to be that way. For example, a <code>typedef</code> or type alias
 * declaration that is only used within a class's member functions should be
 * declared in the private section of the class. If the client code needs to use
 * the declaration, place the it in the public section of the class. Don't place
 * it at the top-level of the header file unless both multiple classes and the
 * client code needs to refer to it. A similar rule applies to <code>enum</code>
 * types, functions, and function object classes such as ordering relations.
 * These rarely need to be declared at the top level of a header file.</li>
 * <li>Every header file should <code>#include</code> every other header file
 * that it requires to compile correctly, but no more. Consider a headerfile
 * <code>A.hpp</code> where class <code>A</code> is declared. If another class
 * or structure type <code>X</code> is used as a member variable of
 * <code>A</code>, then you must <code>#include X.hpp</code> in
 * <code>A.hpp</code> so that the compiler knows how large the <code>X</code>
 * member is. Similarly, if a class <code>A</code> inherits from class
 * <code>X</code> which is declared in <code>X.hpp</code>, then you must
 * <code>#include X.hpp</code> in <code>A.hpp</code>, so that the compiler knows
 * the full contents of an <code>A</code> object. Do not <code>#include</code>
 * header files that only the source file code needs. For example
 * <code>cmath</code> or <code>algorithm</code> is usually needed only by the
 * function definitions in the <code>.cpp</code> file - <code>#include</code> it
 * in <code>.cpp</code> file, not in the <code>.hpp</code> file.</li>
 * <li> If an incomplete declaration of a type <code>X</code> will do, use it
 * instead of <code>#include</code>ing its header <code>X.hpp</code>. If another
 * <code>struct</code> or <code>class</code> type <code>X</code> appears only as
 * a pointer or reference type in the contents of a header file, or as a
 * parameter type or returned value in a function declaration, then you should
 * not <code>#include X.hpp</code>, but just place an incomplete declaration of
 * <code>X</code> (also called a "forward" declaration) near the beginning of
 * the header file, as in:
 * @code
 * 		class X;
 * @endcode
 * See this
 * <a href="http://umich.edu/~eecs381/handouts/IncompleteDeclarations.pdf"
 * target="_blank">handout</a> for more discussion of this powerful and
 * valuable technique. Two important points:
 * <ul>
 * <li>If your header file has a function definition that requires a complete
 * declaration of <code>X</code>, you can almost always eliminate the need by
 * moving that function definition to the <code>.cpp</code> file, so that the
 * header file only has the function declaration. Now you can use only an
 * incomplete declaration in the header file.</li>
 * <li>The Standard library has a header of incomplete declarations that often
 * suffices for the <code>iostream</code> library, named <code>iosfwd</code>.
 * You should <code>#include <iosfwd></code> whenever possible, because the
 * <code>iostream</code> header file is extremely large (giant templates!), and
 * unless you must have function definitions in the header file that use
 * iostream operators, there is no reason to include <code>iostream</code>.</li>
 * </ul></li>
 * <li> The content of a header file should compile correctly by itself. A
 * header file should explicitly <code>#include</code> or forward declare
 * everything it needs. Failure to observe this rule can result in very puzzling
 * errors when other header files or <code>#include</code>s in other files are
 * changed. Check your headers by compiling (by itself) a <code>test.cpp</code>
 * that contains nothing more than
 * @code
 *		#include “A.hpp”
 * @endcode
 * It should not produce any compilation errors. If it does, then something has
 * been left out - something else needs to be included or forward declared. Test
 * all the headers in a project by starting at the bottom of the include
 * hierarchy and work your way to the top. This will help to find and eliminate
 * any accidental dependencies between header files.</li>
 * <li>The <code>A.cpp</code> file should first <code>#include</code> its
 * <code>A.hpp</code> file, and then any other headers required for its code.
 * Always <code>#include A.hpp</code> first to avoid hiding anything it is
 * missing that gets <code>#include</code>d by other header files. Then, if
 * <code>A</code>'s implementation code uses <code>X</code>, explicitly
 * <code>#include X.hpp</code> in <code>A.cpp</code>, so that A.cpp is not
 * dependent on <code>X.hpp</code> accidentally being <code>#include</code>d
 * somewhere else. There is no clear consensus on whether <code>A.cpp</code>
 * should also <code>#include</code> header files that <code>A.hpp</code> has
 * already included. Here are two suggestions:
 * <ul>
 * <li>If the <code>X.hpp</code> file is a logically unavoidable requirement for
 * the declaration in <code>A.h</code> to compile, then <code>#include</code>ing
 * it in <code>A.cpp</code> is redundant, since it is guaranteed to be included
 * by <code>A.hpp</code>. So it is OK to not <code>#include X.hpp</code> in
 * <code>A.cpp</code>, and will save some compiler time (the compiler won’t have
 * to find and open the <code>.hpp</code> file twice).</li>
 * <li>Always <code>#include</code>ing <code>X.h</code> in <code>A.cpp</code> is
 * a way of making it clear to the reader that we are using <code>X</code>, and
 * helps make sure that <code>X</code>’s declarations are available even if the
 * contents of <code>A.h</code> changes due to the design changes. For example,
 * maybe we had a <code>Thing</code> member of a <code>class</code> at first,
 * then changed it to a <code>Thing*</code>, but still used members of
 * <code>Thing</code>s in the implementation code. The <code>#include</code> of
 * <code>Thing.hpp</code> saves us a compile failure. So it is OK to redundantly
 * <code>#include X.hpp</code> in <code>A.cpp</code>. Of course, if
 * <code>X</code> becomes completely unnecessary, all of the
 * <code>#include</code>s of <code>X.h</code> should be removed.</li></ul>
 * </li>
 * <li>Explicitly <code>#include</code> the headers for all Standard Library
 * facilities used in a <code>.cpp</code> file. The C++ Standard does not say
 * which Standard Library header files must be included by which other Standard
 * Library headers, so if you leave one out, the code may compile successfully
 * in one implementation, but then fail in another. (This is a gap in the
 * Standard, justified weakly as giving implementers more freedom to optimize.)
 * </li>
 * <li>Never <code>#include</code> a <code>.cpp</code> file for any reason!
 * <code>.cpp</code> files are normally separately compiled,
 * so if you <code>#include</code> a <code>.cpp</code> file, you have to somehow
 * tell people not to compile this one <code>.cpp</code>
 * file out of all the others. Furthermore, if they miss this hard-to-document
 * point, they get really confused because compiling this sort of odd file
 * typically produces a million error messages, making people think something
 * mysterious is fundamentally wrong with your code or how they installed it.
 * Conclusion: If it can’t be treated like a normal header or source file, don't
 * name it like one!</li>
 * </ol>
 *
 * @section nameing Naming
 *
 * <ol>
 * <li>Take names very, very seriously. They are a major way for you to
 * communicate your design intent to the next person who will touch your code!!!
 * </li>
 * <li>Use CamelCase for class names. For example, <code>CircleDetector</code>.
 * </li>
 * <li>Use mixedCase for function and variable names. For example,
 * <code>detectCircle</code>.</li>
 * <li>Preprocessor symbols defined with <code>#define</code> must be all upper
 * case:
 * @code
 * 		#define MAAV_MEMORY_STREAM_HPP
 * @endcode</li>
 * <li>Files with a class should be named after the name of the class. For
 * example, <code>CircleDetector.hpp</code> and <code>CircleDetector.cpp</code>.
 * </li>
 * <li>Files with procedural (class-less) functions should have a name that
 * describes what it does. For example, <code>math.hpp</code> and
 * <code>math.cpp</code>.</li>
 * <li>Do <i>not</i> start variable or function names or <code>#define</code>
 * symbols with underscores. Leading underscores are reserved for the C/C++
 * preprocessor, compiler, and library implementation's internal symbols and
 * names. Break this rule, and you risk name collisions leading to confusing
 * errors. (The actual rules on reserved leading underscore names is somewhat
 * complex; it is simplified here because there is no good reason to take a
 * chance by pushing the envelope.) </li>
 * <li>Do not use cute or humorous names, especially if they don't help
 * communicate the purpose of the code.</li>
 * <li>Don't include implementation details such as variable type information in
 * variable names. Emphasize purpose instead. If you need to change the name of
 * a variable when you change the type of that variable, then you are violating
 * this rule! </li>
 * <li>Use variable names that do not have to be documented or explained. Longer
 * is usually better:
 * @code
 *		int x = 10;		//Bad
 *		int ngb = 10;	//Bad
 *		int numGreenBalls = 10;	//Good
 * @endcode
 * Single letter conventional variable names are OK for very local, temporary
 * purposes:
 * @code
 *		for (int i = 0; i < 10; i++)
 *			;	//null statement
 * @endcode
 * However, don't use easily confused single letter names, such as
 * <code>I</code>(upper case i), <code>l</code> (lower case l), <code>O</code>
 * (upper case o), <code>0</code> (digit zero), etc. Never reply on the font.
 * </li>
 * <li>Names for named constants should be all uppercase:
 * @code
 *		const double PI = 3.141592653589;
 * @endcode</li>
 * <li><code>typedef</code> names should end with <code>_t</code>.</li>
 * </ol>
 *
 * @section built_in_types Usage of Built-In Types
 *
 * This section covers guidelines on the usage of built-in types, or primitives.
 *
 * @subsection general General Guidelines
 *
 * <ul>
 * <li>Always initialize a primitive before use.</li>
 * </ul>
 *
 * @subsection numeric Numeric Types
 *
 * <ol>
 * <li>Avoid declaring or using <code>unsigned</code> integers; they are
 * seriously error prone and have no compensating advantage. Just try
 * subtracting a larger <code>unsigned</code> value from a smaller
 * <code>unsigned</code> value. If a number should never be negative, either
 * test it explicitly or document it as an invariant with an assertion. <br />
 * However, note that when bitwise manipulations need to be done, using
 * <code>unsigned int</code>s for the bit patterns may be more consistent across
 * platforms than signed <code>int</code>s.<li>
 * Prefer <code>double</code>s over <code>float</code>s. <code>double</code>s
 * are nearly always precise enough for serious computations; <code>float</code>s
 * may not be. Use <code>float</code>s only when memory space needs to be saved
 * (or if required by an API). </li>
 * Do not assume that two <code>float</code>s or <code>double</code>s will
 * compare equal even if mathematically they should. Instead, your code should
 * test for a range of values rather than strict equality.</li>
 * </ol>
 *
 * @subsection string_const String Constants
 *
 * Declare and define string constants as <code>const</code> pointers to
 * <code>const</code> characters initialized to string literals rather than
 * initialized arrays of <code>char</code> or <code>const std::string</code>
 * variables:
 * @code
 *		// Bad
 *		const char message[] = "Follow this style guide!";
 *			// When this instruction is executed, the computer creates
 *			// message as an array that is just big enough to hold the
 *			// string literal, which is already in memory, and then copies
 *			// the string literal into message. This wastes both time and
 *			// memory space.
 *
 *		// Bad
 *		const std::sting message("Follow this style guide!");
 *			// Requires extra storage for message's internal array, plus
 *			// time to allocate this internal array and copy the literal
 *			// into it. Run-time memory footprint is at least twice as
 *			// large as the string literal.
 *
 *		// Good
 *		const char* const message = "Follow this style guide!";
 * @endcode
 *
 * @subsection enum enum Types
 *
 * <ol>
 * <li>Prefer using <code>enum class</code>es over C-style <code>enum</code>s
 * because <code>enum class</code>es are much better behaved. For example, there
 * is no implicit conversions between an <code>enum class</code> and an
 * <code>int</code>. Also, the enumerated values inside an
 * <code>enum class</code> are scoped, so the traditional all-upper-case names
 * used in C-style <code>enum</code>s are unnecessary and distracting; use names
 * that are clear and simple instead:
 * @code
 *		enum class Fruit {APPLE, ORANGE, PEAR}; //Bad
 *		enum class Fruit {Apple, Orange, Pear}; //Good
 * @endcode</li>
 * <li>Use an enumerated type instead of arbitrary numeric code values.</li>
 * <li>Use the default for how <code>enum</code> values are assigned by the
 * compiler:
 * @code
 *		// Bad
 *		enum class Fruit {Apple = 0, Orange, Pear};
 *			// Why? This is the default! Are you confused or what?
 *
 *		// Bad
 *		enum class Fruit {Apple = 1, Orange = 2, Pear = 0};
 *			// What? Why? Is there something wrong with you?
 *
 *		// Good
 *		enum class Fruit {Apple, Orange, Pear};
 * @endcode
 * </li>
 *
 * @section code_strut Code Structure
 *
 * This section covers guidelines on the how to structure your code.
 *
 * @subsection variables Variables
 *
 * <ol>
 * <li>Declare variables in the narrowest scope possible, and at the point where
 * they can given their first useful value.</li>
 * <li>Named constants or "Magic Numbers:"
 * <ul>
 * <li>Numerical or string constants that are "hard coded" or "magic numbers"
 * that are written directly in the code are almost always a bad idea,
 * especially if they appear more than once.
 * @code
 *		// Bad
 * 		double horizontal = 1.33 * vertical;
 *
 *		// Good
 *		const double ASPECT_RATIO = 1.33;
 *		double horizontal = ASPECT_RATIO * vertical;
 * @endcode</li>
 * <li>However, if a value if set "by definition" and can't possibly be anything
 * else, or if a value has no meaning or conventional name, then making that
 * value a named constant can be unnecessary:
 * @code
 *		const int NOON = 12;	// Pointless
 * @endcode</li>
 * <li>A name or symbol for a constant that is a simple synonym for the
 * constant's value is <i>stupid</i>:
 * @code
 *		const double TWO = 2.0;		// Idiotic...
 * @endcode</li>
 * <li>In C++, declaring and initializing a <code>const</code> variable is the
 * idiom for defining a constant. Don’t use <code>#define</code> - it is an
 * inferior approach in C++ because it does not give type information to the
 * compiler.</li>
 * </ul></li>
 * <li>Global Variables (note that the following restrictions do no apply to
 * global named constants):
 * <ul>
 * <li>Avoid using global variables as much as you can! If you are thinking
 * about using a global variable, think again. Then think again. Then think a
 * third time. And if you want to used a global variable, confirm with a senior
 * team member.</li>
 * <li>Do not use global variables to avoid defining function parameters.</li>
 * <li>Global variables are acceptable only when they substantially simplify
 * the information handing in a program. For example, data that is shared
 * between threads is usually global.</li>
 * </ul>
 * </li>
 * </ol>
 *
 * @subsection macros Macros
 *
 * <ol>
 * <li>Do not use macros for anything except for include guards. You may use
 * them for conditional compilation, but before you do so consult with a senior
 * team member.</li>
 * <li>Use the <code>assert</code> macro liberally to help document invariants
 * and help catch programming errors. Note that we waid "programming errors,"
 * not run time error. Also, note that the statement enclosed in an
 * <code>assert</code> macro should not have any side effects.</li>
 * </ol>
 *
 * @subsection control_flow Control Flow
 *
 * <ol>
 * <li>Avoid deeply nested code.
 * @code
 *		// Bad
 *		if (a)
 *		{
 *			// ...
 *			if (b)
 *			{
 *				// ...
 *				if (c)
 *				{
 *					// ...
 *					if (d)
 *					{
 *						//...
 *						// Just when does this excecute?
 *					}
 *				}
 *			}
 *		}
 *
 *		// Good
 *		if (a)
 *		{
 *			// ...
 *		}
 *		else if (b)
 *		{
 *			// ...
 *		}
 *		else if (c)
 *		{
 *			// ...
 *		}
 *		else
 *		{
 *			// ...
 *		}
 * @endcode</li>
 * <li> Prefer using a <code>switch</code> statement to <code>if else if</code>
 * constructions for selecting actions depending on the value of a single
 * variable. These generally results in faster and cleaner code. However, there
 * are several things about <code>switch</code> statements that you should be
 * aware of:
 * <ul>
 * <li><code>switch</code> statements cannot be used if <code>string</code>s or
 * floating point values are being tested.</li>
 * You should always include a <code>default</code> case in a
 * <code>switch</code> statement. This <code>default</code> case should output
 * an error messgae or throw an exception if the control should never reach the
 * <code>default</code>.</li>
 * <li>Remember to terminate each <code>case</code> with a <code>break;</code>
 * statement. If you are arranging a fall-through deliberately, make sure you
 * document it.</li>
 * </ul></li>
 * </ol>
 *
 * @section functions Usage and Design of Functions
 *
 * This section covers guidelines for function usage and design.
 *
 * @subsection when When Should I Use a Function?
 *
 * You should use functions freely to improve the clarity and organization of
 * your code. (Modern computers are quite efficient with function calls, so
 * avoiding function calls at the expense of code clarity is unnecessary.) There
 * are two important cases that calls for the usage of functions especially:
 * <ol>
 * <li><b>Use functions to reduce duplicate code:</b><br />
 * Copy-and-pasting code means copy-and-pasting bugs, and copy-and-pasting bugs
 * means more difficulty when debugging and modifying code. So avoid duplicate
 * code as much as you can by defining a funciton for the duplicated code block.
 * </li>
 * <li><b>Define functions for significant conceptual pieces of work:</b><br />
 * For example, in a spell-checking program, a function that processes a
 * document will need to process each line in the document in sequence. So it
 * makes sense to define a function that processes a line of the document.
 * Similarly, the fuction that processes a line will need to check the spelling
 * of each word in the input line. So it makes sense to define a function that
 * check the spelling of a word.</li>
 * </ol>
 *
 * @subsection when_not When Should I Not Use a Function?
 *
 * We just said that you should use functions freely. However, no freedom is
 * absolute. We do not want you to define rubbish functions. What are rubbish
 * functions? Here is a list:
 * <ol>
 * <li>Functions that are trivial:
 * @code
 *		// Stupid!!!
 *		if (thereIsError())
 *			outputErrorMessage();
 *		// ...
 *		void outputErrorMessage()
 *		{
 *			std::cerr << "There was an error\n";
 *		}
 *
 *		// Good
 *		if (thereIsError())
 *			std::cerr << "There was an error\n";
 * @endcode</li>
 *<li>Functions that simply wraps around a Standard Library function:
 * @code
 *		// Bad
 *		int i;
 *		if(readInt(i))
 *		{
 *			// Do something..
 *		}
 *
 *		bool readInt(int& ir)
 *		{
 *			std::cin >> ir;
 *			return !std::cin;
 *		}
 *
 *		// Good
 *		int i;
 *		if (std::cin >> i)
 *		{
 *			// Do something...
 *		}
 * @endcode</li>
 * <li>Swiss army knife functions:
 * @code
 *		// Bad!!! You can't tell what this function is doing from a call to
 *		// this function. You might end up solving every world problem or
 *		// killing the entire human race.
 *		void useSwissArmyKnife(int operation)
 *		{
 *			if (operation == 1)
 *				// act like a corkscrew
 *			else if (operation == 2)
 *				// act like a screwdriver
 *			else if (operation == 3)
 *				// act like a big knife
 *			else if (operation == 4)
 *				// act like a small knife
 *			else if (operation == 5)
 *				// act like a toe nail clipper
 *			else if (operation == 6)
 *				// make your computer pop gold
 *			else if (operation = 7)
 *				// solve every world problem
 *			else
 *				// kill the human race
 *		}
 * @endcode</li>
 * </ol>
 *
 * @subsection const_args Functions with const Input Arguments
 *
 * There are times when you do not want a function to change the value of one
 * of its input arguments. How this should be done depends on the type of the
 * that input argument:
 * <ol>
 * <li> If the argument is of built-in type. You should simply pass it by value:
 * @code
 *		void func(const int& i);		// Bad.
 *		void func(const int* const i);	// Bad.
 *		void func(int i);				// Good.
 * @endcode
 * The rational behind this is that, for built-in types, passing by value is
 * usually faster than passing by reference or by pointer. And since we are
 * passing the argument by value, using a <code>const</code> is redundant
 * because the function cannot affect the caller's value anyway.</li>
 * <li> If the argument needs complex construction
 * (e.g. <code>std::string</code>), you should pass it by reference-to-const:
 * @code
 *		void func(std::vector<int> v);			// Bad.
 *		void func(const std::vector<int>& s);	// Good.
 * @endcode</li>
 *
 * <li>If the argument is a pointer, you need to think about whether you want
 * the pointer itself to be <code>const</code> or if you want the object that
 * the pointer points to to be <code>const</code>, or both:
 * @code
 *		void func(const int* i); // Passing a pointer to a const int
 *		void func(int const* i); // Also passing a pointer to a const int
 *		void func(int* const i); // Passing a const pointer to a int
 *		void func(const int* const i); // Passing a const pointer to a const int
 * @endcode</li>
 * </ol>
 *
 * @subsection others_func Other Miscellaneous Guidelines
 *
 * <ol>
 * <li> Functions should be short and simple. Avoid functions that are longer
 * than a screenful (~50 lines).</li>
 * <li>Inline short functions for perfomance and clarity:
 * @code
 *		// OK, but the compiler won't optimize this.
 *
 *		// in .hpp file
 *		class Polygon
 *		{
 *		public:
 *			double getArea() const;
 *		private:
 *			double area;
 *		}
 *
 *		// in .cpp file
 *		double Polygon::getArea() const { return area; }
 *
 *
 *		// Better; the compiler will inline getArea() if it can
 *		// in .hpp file
 *		class Polygon
 *		{
 *		public:
 *			double getArea() const { return area; }
 *		private:
 *			double area;
 *		}
 *
 *
 *		// Just as good; the compiler will inline getArea() if it can
 *		// in .hpp file
 *		class Polygon
 *		{
 *		public:
 *			double getArea() const;
 *		private:
 *			double area;
 *		}
 *		inline double Polygon::getArea() const { return area;}
 *		// Notice that this definition needs to be in the header file.
 * @endcode</li>
 * <li> To tell the compiler that you are not using a function parameter in a
 * definition, leave out its name.
 * @code
 *		// this will result in an annoying compiler warning about unused x
 *		void func(int i, double x)
 *		{
 *			// code doesn't use x
 *		}
 *
 *		// No more annoying compiler warnings!
 *		void func(int i, double)
 *		{
 *			// code doesn't use x
 *		}
 * @endcode
 * This techinique should (only) be used when you are subclassing from
 * third-party libraries such as Qt. For example, you might write a class
 * <code>Yichen</code> that inherits from <code>QGraphicsItem</code>:
 * @code
 *		class Yichen : public QGraphicsItem
 *		{
 *		public:
 *			// Overide virtual function in QGraphicsItem
 *			void paint(QPainter*, const QStyleOptionGraphicsItem *,
 *				QWidget *) Q_DECL_OVERRIDE;
 *		};
 * @endcode
 * And if you are not using the <code>QStyleOptionGraphicsItem*</code> and the
 * <code>QWidget*</code> argument, you should write this in the implementation
 * (source) file to supress compiler warnings:
 * @code
 *		void Yichen::paint(QPainter *painter,
 *			const QStyleOptionGraphicsItem *, QWidget *)
 *		{
 *			// Paint Yichen!
 *		}
 * @endcode</li>
 * <li> For functions with different argument types that peform the same type of
 * computation conceptually, instead of naming them differently basing on their
 * argument types, use overloaded functions.
 * @code
 *		// Bad
 *		void doCoolStuffwithInt(int i);
 *		void doCoolStuffwithString(const std::string& s);
 *
 *		// Good
 *		void doCoolStuff(int i);
 *		void doCoolStuff(const std::string& s);
 * @endcode</li>
 * </ol>
 *
 * @section class Usage and Design of User Defined Types
 *
 * Class design is one of the most complex topics in Software Engineering and
 * Object Oriented Design (people write books about it). There is simply no way
 * that we can include a comprehensive guide on class designs in this style
 * guide (I know, you are already complaining that the style guide is too long.
 * Just imagine writing it.) So this section will only talk about some basics.
 * If you want to read more about class usage and design, here is a list of
 * resources that you can refer to:
 *
 * <ol>
 * <li><i>Effective C++</i> by Scott Meyers, which is in the office.</li>
 * <li><i>More Effective C++</i> by Scott Meyers.</li>
 * <li><i>Effective STL</i> by Scott Meyers.</li>
 * <li><i>Effective Modern C++</i> by Scott Meyers, which is in the office.</li>
 * <li><i>Code Complete</i>, 2nd Edition by Steve McConnell.</li>
 * <li><i>Design Patterns: Elements of Reusable Object-Oriented Software</i>
 * by Erich Gamma, Richard Helm, Ralph Johnson, and John Vlissides.</li>
 * <li><i>Head First Design Patterns</i> by Eric Freeman, Bert Bates,
 * Kathy Sierra, Elisabeth Robson</li>
 * <li>EECS 381 <a href="http://umich.edu/~eecs381/handouts/handouts.html"
 * target="_blank">handouts</a> and <a
 * href="http://umich.edu/~eecs381/lecture/notes.html" target="_blank">lecture
 * notes</a>.</li>
 * </ol>
 *
 * When you need to do class desgin while working on MAAV code, always consult
 * a senior member, especially if you are new to class design. Do <i>NOT</i>
 * just assume that your idea if a great idea!!!
 *
 * @subsection general_principles General Principles
 *
 * A class should have a limited, clear, and easliy stated set of
 * responsibilities. If another person asks you what a class you wrote
 * does, you should be able to answer in a few coherent sentences. If you
 * cannot do this, your class is probably poorly designed. Here is a list of
 * the kind classes that you should suspect:
 * <ul>
 * <li>Classes that do things that are not closely related. For example, a
 * class should not deal with IO and also handle Roomba detection.</li>
 * <li>Classes that contains similar code as the code in its client. Either the
 * class code should do the work, or the client code should, not both.</li>
 * <li>Classes that have both getters functions and setters functions for most
 * its member variables.</li>
 * <li>Classes that have getter functions that returns non-<code>const</code>
 * references to a member variable or a pointer to a member variable.</li>
 * <li>Classes that only have one instance. Note that these can be a good idea
 * at times. For example, Singleton classes and classes in the
 * Model-View-Controller pattern (if you don't know what I am not talking about,
 * look at the resources I listed above). However, a lot of times a class that
 * only has one instance are usually bloated and tries to do everything at once.
 * You should avoid those kind of classes.</li>
 * </ul>
 *
 * @subsection access_control Access Control
 *
 * <ul>
 * <li>Always list the members of a class in the order of <code>public</code>,
 * <code>protected</code>, and then <code>private</code> members.</li>
 * <li>The <code>public</code> section of a class should be the interface for
 * the client of the class.</li>
 * <li>The <code>protected</code> section of a class should be the interface
 * for the derived classes of that class.</li>
 * <li>Members that are not <code>public</code> or <code>protected</code>
 * conceptually shoule be declared <code>private</code>.</li>
 * <li>All member variables of a <code>class</code> should be
 * <code>private</code>.</li>
 * </ul>
 *
 * @subsection friends Usage of friends
 *
 * <ul>
 * <li>Limit the usage of <code>friend</code>s to overloading the input (
 * <code>>></code>) and output (<code><<</code>) opeartors. In all other cases,
 * do not use <code>friend</code>s.</li>
 * <li><code>friend</code> declarations should be considered as part of the
 * <code>public</code> interface of the befriended <code>class</code> and should
 * be declared in the same moule.</li>
 * </ul>
 *
 * @subsection structs Usage of structs
 *
 * Use <code>struct</code> instead of <code>class</code> for a simple concrete
 * type all of whose members are conceptually <code>public</code>, and do not
 * use the <code>public</code> or <code>private</code> keywords in the
 * declaration. <code>struct</code>s are especially appropriate if the type is
 * going to be used the same way as a <code>struct</code> in C and can be
 * appropriate even if the type has constructors, member functions, and
 * operators - as long as all members are conceptually <code>public</code>.
 *
 * @subsection construct_copy_destroy Construction, Dopy, and Destruction
 *
 * Here are some general guideline about construtors:
 * <ol>
 * <li>The constructor of a <code>class</code> should ensure that all member
 * variables have meaningful initial values.</li>
 * <li>The constructor of a <code>class</code> should also make sure that all
 * resources needed by an instance of a class are allocated. This is a very
 * basic principle/technique known as Resource Allocation Is Initialization
 * (RAII).</li>
 * <li>As much as possible, initialize member variables in the constructor
 * initializer list rather than in the constructor body. Always list the members
 * in the contructor initializer list in the order in which they are declared in
 * the class.</li>
 * <li>Complex operations such as memory allocation should be placed in the
 * constructor body.</li>
 * <li>Always define a default constructor when it is meaningfull to do so.</li>
 * <li>If you have a single-argument constructor, prefer to define the default
 * constructor using a default parameter value in the same constructor.</li>
 * <li>Declare single-argument constructors as <code>explicit</code> to avoid
 * implicit conversion from the argument type.</li>
 * <li>If the compiler-generated constuctor correctly initialized an object,
 * use the compiler-generated constructor. Don't write code that the compiler
 * already provides.</li>
 * <li>If construction fails, use an exception to inform the client code.</li>
 * </ol>
 *
 * Here are some guidelines about "the Big Five:" destructor, copy constructor,
 * copy assignment operator, move constructor, and move assignment operator.
 * <ol>
 * <li>Always follow "the Rule of Five." Either explicitly define all five of
 * the Big Five, or define none of them and let the compiler supply them.</li>
 * <li>If compiler-generated Big Five do their jobs correctly, then use them.
 * Again, do not write code that thee compiler already provide for you.</li>
 * <li>The destructor of a class should ensure that all resources used by an
 * instance of class is deallocated. This is a counter part to the RAII
 * technique. Some refer to this as Resource Deallocation is Destruction (RDID).
 * <li>If copy and move operations are not meaningful for a class, declare
 * them as <code>=delete</code> to prevent the compiler from supplying
 * them.</li>
 * <li>Follow the swap logic when implementing move constructor and
 * move assignment if possible. See the example at the end of this section for
 * an example of the swap logic.</li>
 * </ol>
 *
 * @subsection others_class Other Miscellaneous Guidelines
 *
 * <ul>
 * <li>Scope <code>typedef</code>s, type aliases, or enumeration types used in a
 * <code>class</code> within the <code>class</code> declaration rather than
 * declare at the top level of a header file.</li>
 * <li>Keep "helper" <code>class</code> or <code>structs</code> out of the
 * header file if possible. If not possible, try declaring the helper inside
 * the class. If even that is not possible, you should probably fix your
 * design.</li>
 * <li>For <code>class</code>-wide constants, either use <code>static</code>
 * member variables or use internally linked constants.</li>
 * <li>When choosing overloaded operators for a class, only overload those
 * operators whose conventional (built-in) meanings are conceptually similar to
 * the operations to be done. Prefer named functions otherwise.</li>
 * <li>Declare member functions <code>const</code> if they do not modify the
 * state of the object.</li>
 * <li>Using <code>const_cast</code> to avoid duplication between
 * the <code>const</code> and non-<code>const</code> version of a member
 * function is allowed. Note that this is the <i>only</i> situation where
 * <code>const_cast</code> is acceptale!!!</li>
 * </ul>
 *
 * @subsection example_class Example
 * Here is an example of a class which illustrates some of the guidelines
 * included in thie section:
 * @code
 *	// This class holds an integer array of any size,
 *	// detects illegal subscripts and throws an exception,
 *	// and implements the rule of 5 by providing copy construction,
 *	// copy assignment, move construction, and move assignment.
 *	// Operator+ is overloaded to sum two Array objects of the same size.
 *
 *	#include <iostream>
 *	using namespace std;
 *
 * 	class Array_Exception
 *	{
 *	public:
 *		Array_Exception (int v, const char * msg) :
 *			value (v), msg_ptr(msg) { }
 *
 *		int value;
 *		const char * msg_ptr;
 *	};
 *
 *	class Array
 *	{
 *	public:
 *		// default constructor creates an empty smart array.
 *		Array() : size(0), ptr(nullptr) { }
 *
 *		// one-argument constructor - argument is desired size of the array
 *		// constructor allocates memory for the array
 *		Array(int size_) : size(size_)
 *		{
 *			if (size <= 0)
 *			{
 *				throw Array_Exception(size, "size must be greater than 0");
 *			}
 *			ptr = new int[size];
 *		}
 *
 *		// copy constructor - initialize this object from another one
 *		Array(const Array& original) : size(original.size), ptr(new int[size])
 *		{
 *			for (int i = 0; i < size; ++i)
 *			{
 *				ptr[i] = original.ptr[i];
 *			}
 *		}
 *
 *		// move constructor - take the guts from the original and leave it in
 *		// a safely destructable state
 *		Array(Array&& original) : size(0), ptr(nullptr)
 *		{
 *			swap(original);
 *		}
 *
 *		// copy the data from rhs into lhs object using copy-swap
 *		Array& operator=(const Array& rhs)
 *		{
 *			Array temp(rhs);
 *			swap(temp);
 *			return *this;
 *		}
 *
 *		// move assignment just swaps rhs with this.
 *		Array& operator=(Array&& rhs)
 *		{
 *			swap(rhs);
 *			return *this;
 *		}
 *
 *		// swap the member variable values of this object with the other
 *		// could use std::swap
 *		void swap(Array& other)
 *		{
 *			int t_size = size;
 *			size = other.size;
 *			other.size = t_size;
 *			int* t_ptr = ptr;
 *			ptr = other.ptr;
 *			other.ptr = t_ptr;
 *		}
 *
 *
 *		// destructor - deallocate resources
 *		~Array()
 *		{
 *			delete[] ptr;
 *		}
 *
 *		int get_size() const
 *		{
 *			return size;
 *		}
 *
 *		// overloaded plus operator returns an Array containing the sum of
 *		// the two
 *		Array operator+(const Array& rhs)
 *		{
 *			// must have the same size
 *			if(size != rhs.get_size())
 *			{
 *				throw Array_Exception(size, "LHS and RHS must have the same size for +");
 *			}
 *			Array result(size);
 *			for(int i = 0; i < size; ++i)
 *			{
 *				result[i] = ptr[i]+rhs[i];
 *			}
 *			return result;
 *		}
 *
 *		// overload the subscripting operator for this class - const version
 *		// this will throw an error if the index is out of range
 *		// (including size == 0 case)
 *		const int& operator[] (int index) const
 *		{
 *			if ( index < 0 || index > size - 1)
 *			{
 *				// throw a bad-subscript exception
 *				throw Array_Exception(index, "Index out of range");
 *			}
 *			return ptr[index];
 *		}
 *
 *		// overload the subscripting operator for this class - nonconst version
 *		// this will throw an error if the index is out of range
 *		// (including size == 0 case)
 *		int& operator[] (int index)
 *		{
 *			// This is the only place where a const_cast is allowed
 *			return const_cast<int&>(static_cast<const Array&>(*this)[index]);
 *		}
 *
 *	private:
 *
 *		int size;
 *		int* ptr;
 *	};
 * @endcode
 *
 * @section cpp_idioms C++ Idioms
 *
 * <ol>
 * <li>In general, prefer C++ facilities over C facilties. For example, use
 * <code>cin</code> and <code>cout</code> instead of <code>scanf</code> and
 * <code>printf</code>; use <code>new</code> and <code>delete</code> instead of
 * <code>malloc</code> and <code>free</code>.</li>
 * <li>Use <code>nullptr</code> instead of <code>NULL</code> or <code>0</code>.
 * </li><li>Use <code>bool</code> instead of zero/non-zero when you need a
 * true/false variable.</li>
 * <li>Do not use the <code>struct</code> keyword except in the declaration of
 * the <code>struct</code> type itself. C++ is not C. You do not have to repeat
 * the <code>struct</code> keyword everywhere you are referring to a
 * <code>struct</code> type.</li>
 * <li>Use <code>typename</code> instead of <code>class</code> in template
 * parameter declarations. A template parameter does not have to be a
 * <code>class</code> type, so <code>typename</code> is clearer:
 * @code
 *		template<class T> void f(int i, T t);		// Bad.
 *		template<typename T> void f(int i, T t);	// Good.
 * @endcode
 * </li><li>Do not use the <code>exit</code> function in C++. Unlike in C,
 * <code>exit</code> in C++ is not equivalent in effect to a <code>return</code>
 * from <code>main</code>: calling <code>exit</code> does not ensure that all
 * relevant destructors are called. Therefore you should only reserve
 * <code>exit</code>s for emergency exits where leaving a mess is the only
 * alternative.</li>
 * <li>Take advantage of the definition of non-zero as <code>true</code> and
 * zero as <code>false</code> when testing pointers. Note that a value of
 * <code>nullptr</code> will test as if it was zero or false.
 * @code
 *		// Clumsy
 *		if(ptr != 0) { }
 *		if(ptr == 0) { }
 *
 *		// Better
 *		if(ptr) { }
 *		if(!ptr) { }
 * @endcode
 * </li><li>Casts:
 * <ul>
 * <li>Casts usually imply that the design is bad and using casts undermines
 * type safety. Try to correct the design if possible.</li>
 * <li>Never use C style cases; always use the appropriate C++ style cast that
 * expresses the intent of the cast. The only exception to this rule is routine
 * numeric conversions. Note that such numeric conversions are usually required
 * to supress compiler warnings while interfacing with Standad Library
 * facilities:
 * @code
 *		// OK
 *		std::vector<int> v{0, 1, 2, 3};
 *		int i = static_cast<int>(v.size());
 *
 *		// Also OK
 *		double var;
 *		int i = (int) var;
 * @endcode</li>
 * </ul></li>
 * <li>Prefer type aliases (with <code>using</code>) over <code>typedef</code>s
 * because a type alias can be a template, while a <code>typedef</code> cannot.
 * However, note that a type alias or a <code>typedef</code> should hide the
 * implementation details of the type. A good example if the Standard Library
 * <code>size_t</code>, which is a <code>typedef</code>. The implementation of
 * <code>size_t</code> is platform dependent, but we don't care. We just know
 * that a <code>size_t</code> always has the same size as that of a pointer.
 * If your type alias or <code>typedef</code>. is not providing this kind of
 * implementation-detail-hiding, you probably shouldn't use it.</li>
 * </ol>
 *
 * @section exceptions Exception Handling
 *
 * <ol>
 * <li>Exceptions should be used to handle run-time errors only. Run-time errors
 * refer to errors caused by events outside the programmer's control. Examples
 * include when the user enters an invalide command, when the system runs out of
 * memory, or when network connection disapper. Use <code>assert</code>s to
 * deal with errors due to a programmer's mistake in logic or coding.</li>
 * <li>Be aware of the C++ philosophy: For faster run speed, the language and
 * Standard components do not do any checks for run-time errors; instead, your
 * code is expected to ensure that an operation is valid before performing it:
 * either by selective checks or careful code design.</li>
 * <li>Explicitly design what a program will do in the case of errors.
 * Avoid designs in which the program simply "muddles through" and attempts to
 * keep going in the presence of run-time errors. It is almost always better to
 * take positive action to either inform the user and/or stop processing and
 * restore to a known state before resuming.</li>
 * <li>Use exceptions to allow a clear and simple separation of error and
 * non-error flow of control, and to clearly assign responsibility for error
 * detection and error handling.</li>
 * <li>Do not use exceptions for a "normal" flow of control. Exception
 * implementations are too inefficient for that purpose; reserve them for true
 * error situations where the program processing has to be stopped, terminated,
 * or redirected in some way.</li>
 * <li>Local try-catches around a single function call are probably poor design
 * compared to normal flow of control techniques. Reserve a local try-catch
 * structure for the situation where you have to do some cleanup in this
 * function before rethrowing the exception to the higher-level handler.</li>
 * <li>Use different exception types to signal different error situations rather
 * than packing different values into the exception objects and testing their
 * contents in the catch.</li>
 * </ol>
 *
 * @section const_correct const Correctness
 *
 * <ol>
 * <li>Use <code>const</code> everywhere that is meaningful and correct. See
 * Item 3 of <i>Effective C++</i> by Scott Meyers (which is in the office)
 * for a complete discussion about the rational of this.</li>
 * <li>If something turns out to be non-<code>const</code> when it is supposed
 * to be <code>const</code>, try to correct the design rather than patch the
 * error using <code>const\_cast</code></li>
 * <li>Don't declare thing that change as <code>const</code>!!!</li>
 * <li>Do not use <code>mutable</code> to fake constness of a member
 * function.</li>
 * </ol>
 *
 * @section std_lib Usage of the Standard Library
 *
 * @subsection stl_general General Principles
 * You should know what the Standard Library facilities do, and should prefer
 * them over your own DIY functions and classes. Why? Well, first of all, why
 * waste time writing and debugging your own code when a solution is ready for
 * you to use? Secondly, the Standard Library facilities are usually well
 * optimized for the platform. Hence despite how smart you are, your code will
 * most likely be slower than the Standard Library. So don't recode the wheel!
 *
 * Also, trust that the Standard Library facilities will do their job correctly.
 * More specifically, do not write code that only makes sense if the Standard
 * Library is defective:
 * @code
 *		// Bad
 *		std::string s = ""; // let's make sure the string is empty!
 *		cin >> s;
 *		// check and return an error code just in case this failed somehow
 *		if(cin.fail())
 *		{
 *			return 1;
 *		}
 *		// check that we read some characters
 *		if(s.size() <= 0)
 *		{
 *			return 1;
 *		}
 *		// looks like we can use the contents of s now
 *
 *		// Good
 *		std::string s; // automatically initialized to the empty string
 *		cin >> s; // will always succeed unless something is grossly wrong
 *		// s is good to go
 * @endcode
 *
 * @subsection string Using std::string
 *
 * <ol>
 * <li>When appending a string <code>a</code> to another string <code>s</code>,
 * prefer <code>s += a;</code> over <code>s = s + a;</code>. The rational behind
 * this is that the second statement will create a temporary string to hold
 * <code>s + a</code> and will then copy this value into <code>s</code>. On the
 * other hand, the first statement will merely expand <code>s</code> and copy
 * the value of <code>a</code> into the expanded memory. Thus the first
 * statement is more efficient than the second one.</li>
 * <li>Avoid using <code>std::string::compare</code> when the regular comparison
 * operators work:
 * @code
 *		// Bad
 *		if (!str.compare("Hello") { } // Difficult to understand
 *
 *		// Good
 *		if (str == "Hello")	{ }	// Clear and obvious
 * @endcode
 * </li></ol>
 *
 * @subsection containers Using Containers (and Arrays)
 *
 * <ol>
 * <li>Use the <code>empty()</code> member function to check a container for
 * being empty rather than comparing the <code>size()</code> to zero:
 * @code
 *		// Bad
 *		if (container.size() == 0) { }
 *
 *		// Good
 *		if (container.empty()) { }
 * @endcode</li>
 * <li>Write <code>for</code> statements for containers (and arrays) in their
 * conventional form if possible:
 * @code
 *		// Good - the conventional, most common form
 *		for(i = 0; i < n; i++)		// correct for almost all cases
 *
 *		// Bad
 *		for(i = 1; i <= n; i++)		// confusing - what is this about?
 *		for(i = n; i > 0; i--)		// better be a good reason for this!
 *		for(i = -1; i <= n; i++)	// totally confusing!
 * @endcode</li>
 * <li>In for loops with iterators, use the pre-increment operator, rather than
 * the post-increment operator. The reason for this is that the post-increment
 * operator must copy and hold a value for returning; if unused, the compiler
 * may not be able to optimize it away. The pre-increment operator does not have
 * this problem and is usually more efficient than then post-increment operator.
 * @code
 *		// Bad
 *		for(list<Thing>::iterator it = things.begin(); it != things.end(); it++)
 *		// Good
 *		for(list<Thing>::iterator it = things.begin(); it != things.end(); ++it)
 * @endcode</li>
 * <li>When iterating through a container (or array), if possible arrange the
 * iteration code to handle the empty case automatically instead of as a
 * separate special case. The iterator interface is designed to do this already,
 * so don't write redundant code like this:
 * @code
 *		if(container.empty())
 *			return;
 *		for(auto iter = container.begin(); iter != container.end(); ++iter)
 *		{
 *			//do stuff
 *		}
 * @endcode
 * Instead, do this:
 * @code
 *		for(auto iter = container.begin(); iter != container.end(); ++iter)
 *		{
 *			// do stuff
 *		}
 * @endcode</li>
 * </ol>
 *
 * @subsection stl Using the STL
 *
 * Often when using STL algorithms, you will need to "pass a function" to the
 * algorithm. There are size different ways to do this: use an ordinary function
 * and pass a pointer to that function, use <code>std::bind</code>, use
 * <code>mem_fn</code>, using a lambda expression, define a custom function
 * object, or use <code>std::function<></code>. Here are some guidelines about
 * how to choose between them:
 * <ol>
 * <li>If the relevant function is already defined, then use the following:
 * <ul>
 * <li>If it is an ordinary function with no extra parameters, simply use it
 * directly.</li>
 * <li>If it is a member function with no parameters, simply wrap it with
 * <code>mem\_fn</code>.</li>
 * <li>If it is either an ordinary function with extra parameters, or a member
 * function with extra parameters, use <code>bind</code> to specify the
 * additional parameters.</li>
 * </ul></li>
 * <li>If the code is not already in a defined function, then do the following:
 * <ul>
 * <li>If the code is short and simple and needed in only one place, define it
 * in-place with a lambda expression in the algorithm call and use whitespace
 * and indentation to make sure it is easy to read. If the lambda expression
 * is too long or complex to be readable when written in place, then do not use
 * a lambda expression.</li>
 * <li>If the code is complex, or it needs to be used in more than one place,
 * either define a function or custom function object class to contain it.</li>
 * <li>If you need to store state during the algorithm execution, use a custom
 * function object class that has member variables, not clumsy and limited
 * ordinary functions with <code>static</code> or global variables.</li>
 * </ul></li>
 * <li>Use <code>std::function<></code> only if you really need to store
 * callable objects of variable or unknown type and have good reasons not to use
 * a simpler, lighter-weight mechanism.</li>
 * </ol>
 *
 * @section dynamic_mem Usage of Dynamic Memory
 *
 * <ol>
 * <li>Avoid using dynamic memory when a varible on the stack would work just as
 * well.</li>
 * <li>When using dynamic memory, all allocated memory must be deallocated by
 * the program before terminating. In other words no memory leak is
 * allowed.</li>
 * <li>Remember to used <code>delete[]</code> if you are deleting a dynamically
 * allocated array.</li>
 * <li>Note that in Standard C++, the <code>new</code> operator will throw a
 * <code>std::bad_alloc</code> exception if it fails. Hence you do not have to
 * check whether the returned pointer is valid or not.</li>
 * </ol>
 *
 * @section remarks Remarks
 * This style guide borrowed heavily from the
 * <a href="http://umich.edu/~eecs381/handouts/C++_Coding_Standards.pdf"
 * target="_blank">EECS 381 C++ Coding Style Guide</a>.
 */

#endif /* MAAV_CTRL_HPP */
