#include "MyForm.h"

using namespace System;
using namespace System::Windows::Forms;

[STAThread]
void main(array<String^>^ argv) {
    Application::EnableVisualStyles();
    Application::SetCompatibleTextRenderingDefault(false);

    Test::MyForm form; //WinFormsTest - ��� ������ �������
    Application::Run(% form);
}